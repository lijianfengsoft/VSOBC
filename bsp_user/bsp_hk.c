/***************************************************************************
*  功能：
*  版本：
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/18 23:02 
*****************************************************************************/               
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#include "ff.h"

#include "driver_debug.h"
#include "bsp_ds1302.h"

#include "bsp_hk.h"
#include "bsp_hklist.h"

HK_Store_t		hk_frame;


uint32_t hk_down_cnt;

FIL fd = { 0 };
static unsigned int fd_timestamp = 0;	//创建hk文件下一个文件时，以此时间撮为文件名，每次获取不同的时间
static int fs_ok = 0;					//创建文件是否成功标志 0：失败    1：成功
static int fd_count = 0;				//一个文件中存入的遥测帧计数

QueueHandle_t obc_hk_queue;
QueueHandle_t eps_hk_queue;

uint16_t  hk_frame_index = 0;      //used for up down index from here to start

HK_Fifo_t hk_main_fifo 		__attribute__((section(".exbss"), used));
HK_Fifo_t hk_append_fifo 	__attribute__((section(".exbss"), used));

static HK_Store_t hk_store 	__attribute__((section(".exbss"), used));

/*hk_fifo初始化*/
void HK_fifoInit(HK_Fifo_t *Q)
{
	Q->front = 0;
	Q->rear = 0;
	Q->bufferCount = 0;
}


/**************************************************
 *  功能:将主副帧数据放入hk_fifo中
 *  日期:2018/1/19 10:05
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
uint8_t HK_fifoIn(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt) 
{
	uint32_t length = 0;

	if (opt == HK_FRAME_MAIN)
	{
		length = HK_MAIN_LENGTH;
	}
	if (opt == HK_FRAME_APPEND) 
	{
		length = HK_APPEND_LENGTH;
	}
	if (((Q->rear + 1) % HK_FIFO_BUFFER_CNT == Q->front) || (Q->bufferCount == (HK_FIFO_BUFFER_CNT - 1)))
		return HK_FIFO_FULL;

	Q->rear = (Q->rear + 1) % HK_FIFO_BUFFER_CNT;

	/*将以pdata为首地址，以主或副帧为长度，将数据复制到HK—FIFO中*/
	memcpy(Q->frame[Q->rear], pdata, length);
	Q->bufferCount++;

	hk_frame_index = (hk_frame_index + 1) % HK_FIFO_BUFFER_CNT;

	return(HK_FIFO_OK);
}

/**************************************************
 *  功能:将hk_fifo中的数据取出
 *  日期:2018/1/19 10:06
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
uint8_t HK_fifoOut(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt) 
{
	uint32_t length = 0;

	if (opt == HK_FRAME_MAIN) 
	{
		length = HK_MAIN_LENGTH;
	}
	if (opt == HK_FRAME_APPEND) 
	{
		length = HK_APPEND_LENGTH;
	}
	if ((Q->front == Q->rear) && (Q->bufferCount == 0))
		return(HK_FIFO_EMPTY);
	else 
	{
		Q->front = (Q->front + 1) % HK_FIFO_BUFFER_CNT;
		memcpy(pdata, Q->frame[Q->front], length);
		Q->bufferCount--;
		return(HK_FIFO_OK);
	}
}

/**************************************************
 *  功能:在hk_fifo中以中值法查找utc为timevalue的一组数据
 *  日期:2018/1/19 10:09
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
uint16_t hk_fifo_find(const HK_Fifo_t *Q, uint32_t timevalue)
{
	uint16_t index = 0;
	uint16_t low = 0, high = hk_frame_index, mid;

	mid = (low + high) / 2;

	while (low <= high)
	{
		if (timevalue < ((HK_Main_t *)&(Q->frame[mid]))->obc.utc_time)
		{
			high = mid - 1;
		}
		else if (timevalue > ((HK_Main_t *)&(Q->frame[mid]))->obc.utc_time)
		{
			low = mid + 1;
		}
		else
		{
			index = mid;
			break;
		}
		mid = (low + high) / 2;
	}

	return index;
}

/**************************************************
 *  功能:首次调用时创建hk文件夹，并以此时utc时间撮在此文件夹下创建一个文件
 *  日期:2018/1/19 10:20
 *  输入:
 *  输出:
 *  注意点:创建的文件并没有关闭，只是用f_sync刷新缓存信息
 *************************************************/
int hk_store_init(void) 
{
	static uint32_t hk_file_init = 0;

	/*若初始化标志位0，则创建hk文件夹*/
	if (hk_file_init == 0)
	{
		f_mkdir("0:hk");
		hk_file_init++;
	}

	/* 获取当前UNIX时间的时间戳*/
	timestamp_t time_now;
	clock_get_time(&time_now);
	fd_timestamp = time_now.tv_sec;

	driver_debug(DEBUG_HK, "fd_timestamp is %d\r\n", fd_timestamp);

	char path[50];
	sprintf(path, "0:hk/%u.txt", fd_timestamp);
	driver_debug(DEBUG_HK, "Create and Open file %s\r\n", path);

	/*在/hk目录下创建以时间戳命名的.txt文件*/
	int result = (f_open(&fd, path, FA_READ | FA_WRITE | FA_CREATE_ALWAYS));
	if (result != FR_OK) 
	{
		driver_debug(DEBUG_HK, "hk_store_init fail to Creates:%s  Result:%d \r\n", path, result);
		fs_ok = 0;
		return -1;
	}

	/* 将缓存写入物理磁盘，防止意外掉电损坏文件系统 */
	f_sync(&fd);

	/*将文件名加入遥测列表*/
	if (!hk_list_insert(&hk_list, fd_timestamp)) 
	{
		driver_debug(DEBUG_HK, "Failed to insert\r\n");
		fs_ok = 0;
		return -1;
	}

	fs_ok = 1;

	return 0;
}


/**************************************************
 *  功能:从遥测主帧辅帧FIFO中读出遥测，填入hk_store结构体
 *  日期:2018/1/19 10:25
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void hk_out(void)
{
	HK_fifoOut(&hk_main_fifo, (unsigned char *)&(hk_store.main_frame), (uint8_t)HK_FRAME_MAIN);
	HK_fifoOut(&hk_append_fifo, (unsigned char *)&(hk_store.append_frame), (uint8_t)HK_FRAME_APPEND);
}

int hk_store_add(void)
{
	hk_out();

	/* 将从FIFO读出的遥测存入TF卡 */
	if (fs_ok)
	{
		UINT bww;

		/*尝试写入TF卡*/
		int result = f_write(&fd, &(hk_store), sizeof(HK_Store_t), &bww);

		/*若写入失败，则关闭文件，置初始化成功标志为0*/
		if (result != FR_OK || bww == 0)
		{
			driver_debug(DEBUG_HK, "Failed to write HK to SD-Card\r\n");
			f_close(&fd);
			fs_ok = 0;
			return -1;
		}

		f_sync(&fd);
	}
	/* 若没有文件存储初始化 */
	else
	{
		hk_store_init();
		return 0;
	}

	/* 一个文件中最多存储HK_FILE_MAX_COUNT条遥测信息 */
	if (++fd_count > HK_FILE_MAX_COUNT) {
		f_close(&fd);
		fd_count = 0;
		hk_store_init();
	}

	return 0;
}



/**************************************************
 *  功能:由hklist中的值找到对应sd卡中文件，遍历列表，删除没有对应文件的列表项和文件大小不符合的列表项
 *  日期:2018/1/19 10:57
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void vTelemetryFileManage(void * paragram)
{
	char hk_path[50];
	FILINFO fno = { 0 };

	hkListNode_t *pxIterator = NULL;
	hkList_t * pxList = (hkList_t *)paragram;

	/* 把遥测列表的首列表项赋给pxIterator */
	pxIterator = (hkListNode_t *)&pxList->xListEnd;

	while (1)
	{
		/*清空路径*/
		memset(hk_path, '\0', sizeof(hk_path));

		/*指针指向列表中下一个列表项*/
		pxIterator = pxIterator->pxNext;

		/* 如果首列表项与尾列表项相同，列表遍历完成 */
		if (pxIterator == (hkListNode_t *)(&(pxList->xListEnd)))
			break;

		sprintf(hk_path, "0:hk/%u.txt", pxIterator->TimeValue);
		driver_debug(DEBUG_HK, "Check if the file exists:%s\r\n", hk_path);

		/* 检查文件是否存在，若不存在则将文件名从列表中移除 */
		if ((f_stat(hk_path, &fno)) != FR_OK)
		{
			driver_debug(DEBUG_HK, "File does not exist: %s\r\n", hk_path);
			hk_list_remove(pxIterator);
			continue;
		}

		/* 若文件存在，文件小于HK_Store_t结构体大小的文件从列表中移除 */
		if (fno.fsize < sizeof(HK_Store_t))
		{
			driver_debug(DEBUG_HK, "File size error: %s\r\n", hk_path);
			hk_list_remove(pxIterator);
			continue;
		}
	}
}


void hk_collect_no_store(void)
{

	hk_down_cnt++;

	/*星务计算机本地遥测*/
	obc_hk_get_peek(&hk_frame.main_frame.obc);

	/*电源系统遥测获取*/
	eps_hk_get_peek(&hk_frame.main_frame.eps);

	/*测控分系统遥测获取*/
	ttc_hk_get_peek(&hk_frame.main_frame.ttc);

	/*姿控分系统遥测获取*/
	adcs_hk_get_peek(&hk_frame.append_frame.adcs_hk);
}

int obc_hk_get_peek(obc_hk_t * obc)
{
	if (obc_hk_queue == NULL)
		return pdFALSE;
	return xQueuePeek(obc_hk_queue, obc, 0);
}

int eps_hk_get_peek(eps_hk_t *eps)
{
	EpsAdcValue_t eps_hk;
	if (eps_hk_queue == NULL)
		return pdFALSE;

	if (xQueuePeek(eps_hk_queue, &eps_hk, 0) != pdTRUE)
		return pdFALSE;

	/*电源系统遥测*/
	eps->temp_batt_board[0] = eps_hk.BatTemp[0];
	eps->temp_batt_board[1] = eps_hk.BatTemp[1];
	for (int i = 0; i < 4; i++)
		eps->temp_eps[i] = eps_hk.EpsTemp[i];
	for (int i = 0; i < 6; i++)
	{
		eps->sun_c[i] = eps_hk.In_SunC[i];
		eps->sun_v[i] = eps_hk.In_SunV[i];
	}
	eps->out_BusC = eps_hk.Out_BusC;
	eps->out_BusV = eps_hk.Out_BusV;
	eps->UV_board_C = eps_hk.Out_ComC;

	for (int i = 0; i < 6; i++)
		eps->Vol_5_C[i] = eps_hk.Out_BranchC[i];
	for (int i = 0; i < 5; i++)
		eps->Bus_c[i] = eps_hk.Out_BranchC[i + 6];

	return pdTRUE;
}