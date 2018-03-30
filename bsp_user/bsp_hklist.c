/***************************************************************************
*  功能：hklist的创建
*  版本：
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/17 13:07 
*****************************************************************************/                                                    
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_hklist.h"
#include "ff.h"
#include "usart.h"
#include "driver_debug.h"
#include "error.h"

#include "stdio.h"
#include "stdlib.h"
#include "time.h"


hkList_t  hk_list = { 0 };


/**************************************************
 *  功能:初始化列表项
 *  日期:2018/1/17 13:26
 *  输入:列表指针
 *  输出:
 *  注意点:此时列表项为空，有一个尾项，尾项为最小项,前后指针指向自己
 *************************************************/
void hk_list_init(hkList_t * pxList)
{
	pxList->pxIndex = (hkListNode_t *) &(pxList->xListEnd);

	/*遥测列表初始化时把尾项初始化成0*/
	pxList->xListEnd.TimeValue = 0x00UL;

	/*此时列表中没有列表项，只有一个END节点，它的前后之前指向自己，值大小设为0*/
	pxList->xListEnd.pxNext = (hkListNode_t *) &(pxList->xListEnd);
	pxList->xListEnd.pxPrevious = (hkListNode_t *) &(pxList->xListEnd);

	pxList->uxNumberOfItems = (uint32_t)0U;
}


/**************************************************
 *  功能:从列表中移除列表项，并删除此列表项对应的遥测文件
 *  日期:2018/1/18 22:06
 *  输入:
 *  输出:
 *  注意点:删除的列表项要释放空间
 *************************************************/
uint32_t hk_list_remove(hkListNode_t * pxNodeToRemove) 
{
	hkList_t * const pxList = (hkList_t *)pxNodeToRemove->pvContainer;

	pxNodeToRemove->pxNext->pxPrevious = pxNodeToRemove->pxPrevious;
	pxNodeToRemove->pxPrevious->pxNext = pxNodeToRemove->pxNext;

	/* 若列表的index指向的是将要移除的列表项，则将index指向此列表项的前一项 */
	if (pxList->pxIndex == pxNodeToRemove)
	{
		pxList->pxIndex = pxNodeToRemove->pxPrevious;
	}

	pxNodeToRemove->pvContainer = NULL;
	(pxList->uxNumberOfItems)--;

	char hkpath[25];
	FILINFO fno;
	/* 移除列表项内容所对应的遥测文件 */
	sprintf(hkpath, "0:hk/%u.txt", pxNodeToRemove->TimeValue);

	if (f_stat(hkpath, &fno) != FR_NO_FILE)
	{
		//sprintf(hkpath, "0:hk/%u.txt", pxNodeToRemove->TimeValue);
		/* 删除文件 */
		f_unlink(hkpath);
		driver_debug(DEBUG_HK, " DELETE file: %s \r\n", hkpath);
	}

	vPortFree(pxNodeToRemove);

	return pxList->uxNumberOfItems;
}


/**************************************************
 *  功能:向hklist列表中插入一个节点，按照节点值由大到小的顺序插入
 *  日期:2018/1/18 21:10
 *  输入:hkList_t：hklist的指针		xValueOfInsertion：插入节点的数值
 *  输出:
 *  注意点:在插入前申请了一个节点空间直到删除此节点时才释放
 *************************************************/
uint32_t hk_list_insert(hkList_t * pxList, uint32_t xValueOfInsertion) 
{
	hkListNode_t *pxIterator = NULL;

	hkListNode_t *pxNewListNode = (hkListNode_t *)pvPortMalloc(sizeof(hkListNode_t));

	if (pxNewListNode == NULL)
	{
		driver_debug(DEBUG_HK, "malloc failed\r\n");
		return 0;
	}

	pxNewListNode->TimeValue = xValueOfInsertion;

	/*如果遥测列表的列表项数目大于HK_LIST_NODE_CNT，则移除列表项值最小的一项
	*最小项即为插入时间值最早，为END节点的前一个节点
	*/
	if (++(pxList->uxNumberOfItems) > HK_LIST_NODE_CNT)
	{
		pxIterator = (hkListNode_t *)(pxList->xListEnd.pxPrevious);
		if (pxIterator != (hkListNode_t *) &(pxList->xListEnd))
		{
			hk_list_remove(pxIterator);
		}
	}

	/*从头遍历列表，找到该列表项需插入的位置*/
	for (pxIterator = (hkListNode_t *) &(pxList->xListEnd); xValueOfInsertion < pxIterator->pxNext->TimeValue; pxIterator = pxIterator->pxNext)
	{
		if (pxIterator->pxNext == (hkListNode_t *) &(pxList->xListEnd))
		{
			(pxList->uxNumberOfItems)--;
			return 0;
		}
	}

	pxNewListNode->pxNext = pxIterator->pxNext;
	pxNewListNode->pxNext->pxPrevious = pxNewListNode;
	pxNewListNode->pxPrevious = pxIterator;
	pxIterator->pxNext = pxNewListNode;

	pxNewListNode->pvContainer = (void *)pxList;
	pxList->pxIndex = pxNewListNode;

	return pxList->uxNumberOfItems;
}


/**************************************************
 *  功能:读取文件夹中的每个文件的文件名，来恢复遥测列表
 *  日期:2018/1/18 22:16
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
uint32_t hk_list_recover(void) 
{

	FILINFO fno;
	DIR dir;
	int i = 0;
	uint32_t timevalue = 0;

	/* 尝试打开根目录中的hk文件夹 */
	int result = f_opendir(&dir, "0:hk");

	/* 若成功打开 */
	if (result == FR_OK)
	{
		for (;;)
		{

			result = f_readdir(&dir, &fno);
			if (result != FR_OK)
			{
				/*printf(*/driver_debug(DEBUG_HK, "read directory failed\r\n");
				/*printf(*/driver_debug(DEBUG_HK, "list error ,result is :%u\r\n", result);

				return CMD_ERROR_FAIL;
			}

			if (fno.fname[0] == 0) break;

			printf("%s\r\n", fno.fname);

			timevalue = atol(fno.fname);

			if (!hk_list_insert(&hk_list, timevalue))
			{
				/*printf(*/driver_debug(DEBUG_HK, "Failed to insert\r\n");
			}
			/*printf(*/driver_debug(DEBUG_HK, "Telemetry recover success,file create time: \r\n%s\r\n", ctime(&timevalue));
		}
	}
	else
		/*printf(*/driver_debug(DEBUG_HK, " The directory has not yet been created '0:hk/'\r\n");
	
	f_closedir(&dir);

	return 0;
}

/**************************************************
 *  功能:查找某一列表项
 *  日期:2018/1/18 22:26
 *  输入:
 *  输出:
 *  注意点:若查找的列表中没有此值的列表，则取比此致稍小的一项
 *************************************************/
void * hk_list_find(uint32_t time)
{
	/*从最旧的也就是时间最小的遥测开始找起*/
	hkListNode_t *pxIterator = hk_list.xListEnd.pxPrevious;
	const uint32_t xValueOfFinding = time;

	/*如果要找的时间点小于最旧遥测的时间，则返回最旧的时间点*/
	if (xValueOfFinding < pxIterator->TimeValue)
		return pxIterator;

	while (1)
	{
		if (xValueOfFinding > pxIterator->TimeValue && xValueOfFinding < pxIterator->pxPrevious->TimeValue)
		{
			break;
		}
		pxIterator = pxIterator->pxPrevious;
		if (pxIterator == (hkListNode_t *) &(hk_list.xListEnd)) {
			return NULL;
		}
	}
	return pxIterator;
}