/***************************************************************************
*  功能：CPU内flash的操作
*  版本：V1.0
*  迭代：经测试程序在此时，0x0800CF00之前都存有程序或者数据,根据具体程序大小来判断此位置
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/7 20:14 
*****************************************************************************/                                                    

#include "bsp_cpuflsh.h"
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "delay.h"
#include "usart.h" 

/**************************************************
 *  功能:从指定地址读取指定长度的数据
 *  日期:2018/1/7 20:16
 *  输入:FlashAdrr:待读取缓冲区的首地址
 *		 size：读取的数据长度/字节
 *		 Dest:读取的数据存放区	
 *  输出:错误码（0 成功；1 失败）
 *  注意点:不要超出flash 1M的范围（0x08000000-0x080E0000）
 *************************************************/
u8 STMFLASH_ReadByte(u32 FlashAddr, u8 *Dest, u32 size)
{
	u32 i;
	if (FlashAddr + size > STM32_FLASH_BASE + FLASH_SIZE)
		return 1;
	if (size <= 0)
		return 1;
	for ( i = 0; i < size; i++)
	{
		*Dest++ = *(uint8_t*)FlashAddr++;
	}
	return 0;
}


/**************************************************
 *  功能:获取某个地址所在的flash扇区
 *  日期:2018/1/7 20:24
 *  输入:flash地址
 *  输出:0~11,即addr所在的扇区
 *  注意点:
 *************************************************/
uint16_t STM_GetFlashSector(u32 addr)
{
	if (addr < ADDR_FLASH_SECTOR_1)
		return FLASH_Sector_0;
	else if (addr < ADDR_FLASH_SECTOR_2)
		return FLASH_Sector_1;
	else if (addr < ADDR_FLASH_SECTOR_3)
		return FLASH_Sector_2;
	else if (addr < ADDR_FLASH_SECTOR_4)
		return FLASH_Sector_3;
	else if (addr < ADDR_FLASH_SECTOR_5)
		return FLASH_Sector_4;
	else if (addr < ADDR_FLASH_SECTOR_6)
		return FLASH_Sector_5;
	else if (addr < ADDR_FLASH_SECTOR_7)
		return FLASH_Sector_6;
	else if (addr < ADDR_FLASH_SECTOR_8)
		return FLASH_Sector_7;
	else if (addr < ADDR_FLASH_SECTOR_9)
		return FLASH_Sector_8;
	else if (addr < ADDR_FLASH_SECTOR_10)
		return FLASH_Sector_9;
	else if (addr < ADDR_FLASH_SECTOR_11)
		return FLASH_Sector_10;
	return FLASH_Sector_11;
}

/**************************************************
 *  功能:比较待写入和原数据
 *  日期:2018/1/7 21:22
 *  输入:FlashAdrr:待比较缓冲区的首地址
 *		 CmpBuf：和FLASH进行比较的字符串
 *		 size：比较的数据长度/字节
 *  输出:错误码
 *  注意点:
 *************************************************/
uint8_t STM_CmpFlash(u32 FlashAdrr, uint8_t * CmpBuf, uint32_t size)
{
	uint32_t i;
	uint8_t IsEqu;
	uint8_t cByte;

	if (FlashAdrr + size > STM32_FLASH_BASE + FLASH_SIZE)
		return FLASH_PARAM_ERR;
	if (size == 0)
		return FLASH_IS_EQU;
	IsEqu = 1;												//首先假设相等
	for (i = 0; i < size; i++)
	{
		cByte = *(uint8_t *)FlashAdrr;
		if (cByte != *CmpBuf)
		{
			if (cByte != 0xFF)
				return FLASH_REQ_ERASE;
			else
				IsEqu = 0;
		}
		FlashAdrr++;
		CmpBuf++;
	}
	if (IsEqu == 1)
		return FLASH_IS_EQU;
	else
		return FLASH_REQ_WRITE;
}

/**************************************************
 *  功能:写CPU FLASH
 *  日期:2018/1/7 22:00
 *  输入:FlashAddr：写入的首地址
 *		Src：待写入数据首地址
 *		size：待写入数据长度/字节
 *  输出:错误码 0 正确；1 擦除错误；2 写入错误
 *  注意点:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
 *        写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
 *		  写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
 *        没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写.
 *************************************************/
uint8_t STM_WriteFlash(uint32_t FlashAddr, uint8_t*Src, uint32_t size)
{
	uint32_t i;
	uint8_t Ret;

	if (FlashAddr + size > STM32_FLASH_BASE + FLASH_SIZE)
		return 1;
	if (size <= 0)
		return 1;

	Ret = STM_CmpFlash(FlashAddr, Src, size);
	if (Ret == FLASH_IS_EQU)
		return 0;

	__set_PRIMASK(1);
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	if (Ret == FLASH_REQ_ERASE)
	{
		if (FLASH_EraseSector(STM_GetFlashSector(FlashAddr), VoltageRange_3) != FLASH_COMPLETE)
		{
			FLASH_Lock();
			__set_PRIMASK(0);
			return 1;
		}
	}
	for (i = 0; i < size; i++)
	{
		if (FLASH_ProgramByte(FlashAddr++, *Src++) != FLASH_COMPLETE)
		{
			FLASH_Lock();
			__set_PRIMASK(0);
			return 2;
		}
	}
	FLASH_Lock();
	__set_PRIMASK(0);
	return 0;
}


/**************************************************
 *  功能:擦除Flash中的某一个扇区
 *  日期:2018/1/7 22:13
 *  输入:待擦除扇区内的一个地址
 *  输出:错误码 0 成功； 1 失败
 *  注意点:
 *************************************************/
uint8_t STM_EraseFlash(u32 FlashAddr)
{
	if (FlashAddr > STM32_FLASH_BASE + FLASH_SIZE)
		return 1;
	__set_PRIMASK(1);
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	if (FLASH_EraseSector(STM_GetFlashSector(FlashAddr), VoltageRange_3) != FLASH_COMPLETE)
	{
		FLASH_Lock();
		__set_PRIMASK(0);
		return 1;
	}
	FLASH_Lock();
	__set_PRIMASK(0);
	return 0;
}















