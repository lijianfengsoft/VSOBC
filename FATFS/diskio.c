/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/* 使用FatFs自带的f_mkfs()函数格式化SD卡, 但注意底层函数的正确(disk_ioctl()) */
/*	要不会出现莫名其妙的问题, 比如将你1G的卡格式化为30M.					 */	
/*f_mkfs() 应在 f_mount()之后，否则出现，无工作区的错位。不经意容易犯错。    */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "sdio_sdcard.h"
#include "bsp_ds1302.h" 


#define SD_CARD	 0  //SD卡,卷标为0

		 		 										//初始化磁盘
DSTATUS disk_initialize(
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	u8 res = 0;
	switch (pdrv)
	{
	case SD_CARD://SD卡
		res = SD_Init();//SD卡初始化 
		break;
	default:
		res = 1;
	}
	if (res)return  STA_NOINIT;
	else return 0; //初始化成功
}

//获得磁盘状态
DSTATUS disk_status(
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	return 0;
}

//读扇区
//drv:磁盘编号0~9
//*buff:数据接收缓冲首地址
//sector:扇区地址
//count:需要读取的扇区数
DRESULT disk_read(
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	u8 res = 0;
	if (!count)return RES_PARERR;//count不能等于0，否则返回参数错误		 	 
	switch (pdrv)
	{
	case SD_CARD://SD卡
		res = SD_ReadDisk(buff, sector, count);
		while (res)//读出错
		{
			SD_Init();	//重新初始化SD卡
			res = SD_ReadDisk(buff, sector, count);
			//printf("sd rd error:%d\r\n",res);
		}
		break;
	default:
		res = 1;
	}
	//处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
	if (res == 0x00)
		return RES_OK;
	else return RES_ERROR;
}

//写扇区
//drv:磁盘编号0~9
//*buff:发送数据首地址
//sector:扇区地址
//count:需要写入的扇区数

DRESULT disk_write(
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	u8 res = 0;
	if (!count)return RES_PARERR;//count不能等于0，否则返回参数错误		 	 
	switch (pdrv)
	{
	case SD_CARD://SD卡
		res = SD_WriteDisk((u8*)buff, sector, count);
		while (res)//写出错
		{
			SD_Init();	//重新初始化SD卡
			res = SD_WriteDisk((u8*)buff, sector, count);
			//printf("sd wr error:%d\r\n",res);
		}
		break;
	default:
		res = 1;
	}
	//处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
	if (res == 0x00)
		return RES_OK;
	else return RES_ERROR;
}



//其他表参数的获得
//drv:磁盘编号0~9
//ctrl:控制代码
//*buff:发送/接收缓冲区指针

DRESULT disk_ioctl(
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	if (pdrv == SD_CARD)//SD卡
	{
		switch (cmd)
		{
		case CTRL_SYNC:
			res = RES_OK;
			break;
		case GET_SECTOR_SIZE:
			*(DWORD*)buff = 512;
			res = RES_OK;
			break;
		case GET_BLOCK_SIZE:
			*(WORD*)buff = SDCardInfo.CardBlockSize;
			res = RES_OK;
			break;
		case GET_SECTOR_COUNT:
			*(DWORD*)buff = SDCardInfo.CardCapacity / 512;
			res = RES_OK;
			break;
		default:
			res = RES_PARERR;
			break;
		}
	}
	else 
		res = RES_ERROR;//其他的不支持
	return res;
}
//获得时间
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime(void)
{
	struct ds1302_clock  clock;
	ds1302_clock_read_burst(&clock);
	return	((DWORD)(clock.year - 1980) << 25)
		| ((DWORD)clock.month << 21)
		| ((DWORD)clock.day << 16)
		| ((DWORD)clock.hour << 11)
		| ((DWORD)clock.minutes << 5)
		| ((DWORD)clock.seconds >> 1);;
}


















