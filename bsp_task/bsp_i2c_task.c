/***************************************************************************
*  功能：与I2C相关的任务
*  版本：V 1.0
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/16 22:25 
*****************************************************************************/                                                    

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_pca9665.h"
#include "delay.h"
#include "bsp_i2c_task.h"
#include "error.h"
#include "usart.h"

void i2c_master_task(void *param __attribute__((unused)))
{
	int16_t   i;
	uint8_t data[10];
	for (i = 0; i < 10; i++)
	{
		data[i] = i;
	}
	while (1)
	{
		i2c_master_transaction(0, 0x41, data, 10, NULL, 0, 1000);
		vTaskDelay(2000 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

void i2c_slave_task(void *param __attribute__((unused)))
{
	i2c_frame_t 	*frame = NULL;
	int 			handle = 0;
	uint32_t 		length = 0;
	int             i, flag;

	while (1) 
	{
		flag = i2c_receive(handle, &frame, portMAX_DELAY);
		//if (flag == E_NO_ERR) 
		//{
			//length = frame->len;
			//printf("\r\nI2C receive handle %d, length: %u\n\r", handle, length);
			//for (i = 0; i < length; i++) 
			//{
			//	printf("%x", frame->data[i]);
			//}
		/*}*/
		//printf("\n");
		vPortFree(frame);
	}
}

