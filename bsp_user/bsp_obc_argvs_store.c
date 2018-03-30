/***************************************************************************
*  功能：在复位前在STM32CPUFLASH里存储数据，保存天线等状态,从0x080C0000开始存
*  版本：
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/11 20:28 
*****************************************************************************/      
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_obc_argvs_store.h"
#include "bsp_cpuflsh.h"
#include "bsp_ds1302.h"

obc_save_t obc_save = { 0 };

uint8_t obc_argvs_store(void)
{
	uint8_t res = 1;

	res = STMFLASH_ReadByte(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));
	obc_save.obc_reset_time = clock_get_time_nopara();
	//obc_save.antenna_status = antenna_status;
	//obc_save.hk_down_cnt = hk_down_cnt;
	//obc_save.hk_store_cnt = hk_store_cnt;
	res = STM_WriteFlash(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));

	return res;
}

uint8_t obc_argvs_recover(void)
{
	uint8_t res = 1;
	if (STMFLASH_ReadByte(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save)) == 1)
	{
		//obc_boot_count = 0;
		//obc_reset_time = 0;
		//antenna_status = 0;

		return 1;
	}
	else
	{
		if (obc_save.obc_boot_count == 0xFFFFFFFF || obc_save.obc_boot_count == 0)
		{
			obc_save.obc_boot_count = 1;
		}
		else
		{
			obc_save.obc_boot_count = obc_save.obc_boot_count + 1;
		}
		res = STM_WriteFlash(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));
		if (obc_save.obc_reset_time == 0xFFFFFFFF)
		{
			obc_save.obc_reset_time = 0;
		}
		//if (obc_save.antenna_status != 0 && obc_save.antenna_status != 1 && obc_save.antenna_status != 2)
		//{
		//	obc_save.antenna_status = 0;
		//}

		//if (obc_save.hk_down_cnt == 0xFFFFFFFF)
		//{
		//	obc_save.hk_down_cnt = 0;
		//}

		//if (obc_save.hk_store_cnt == 0xFFFFFFFF)
		//{
		//	obc_save.hk_store_cnt = 0;
		//}
	}
	//obc_boot_count = obc_save.obc_boot_count;
	//obc_reset_time = obc_save.obc_reset_time;
	//antenna_status = obc_save.antenna_status;
	//hk_down_cnt = obc_save.hk_down_cnt;
	//hk_store_cnt = obc_save.hk_store_cnt;
}
