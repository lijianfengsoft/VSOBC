#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "console.h"
#include "command.h"
#include "bsp_i2c_task.h"




#define START_TASK_PRIO  1					//开始任务优先级
#define START_STK_SIZE  128					//开始任务堆栈
TaskHandle_t StartTask_Handler;             //开始任务句柄
void start_task(void *pvParameters);		//开始任务实体函数声明

#define DEBUG_TASK_PRIO  4					//调试任务优先级
#define DEBUG_STK_SIZE  130*4				//调试任务堆栈
TaskHandle_t DebugTask_Handler;             //调试任务句柄

#define I2CSLAVE_TASK_PRIO  3					//I2C从机任务优先级
#define I2CSLAVE_STK_SIZE  130*4				//I2C从机任务堆栈
TaskHandle_t I2CslaveTask_Handler;             //I2C从机任务句柄

#define I2CMASTER_TASK_PRIO  3					//I2C主机任务优先级
#define I2CMASTER_STK_SIZE  130*4				//I2C主机任务堆栈
TaskHandle_t I2CmasterTask_Handler;             //I2C主机任务句柄

int main()
{
		bsp_init();

		xTaskCreate((TaskFunction_t)start_task,           
			        (const char*   )"start_task",         
			        (uint16_t      )START_STK_SIZE,       
			        (void*         )NULL,                  
			        (UBaseType_t   )START_TASK_PRIO, 
			        (TaskHandle_t* )&StartTask_Handler); 
		vTaskStartScheduler();	
}

void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();

	xTaskCreate((TaskFunction_t)debug_console,
				(const char*	)"debug_task",
				(uint16_t		)DEBUG_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)DEBUG_TASK_PRIO,
				(TaskHandle_t*	)&DebugTask_Handler);

	xTaskCreate((TaskFunction_t)i2c_master_task,
				(const char*	)"i2c_master_task",
				(uint16_t		)I2CMASTER_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)I2CMASTER_TASK_PRIO,
				(TaskHandle_t*	)&I2CmasterTask_Handler);

	xTaskCreate((TaskFunction_t)i2c_slave_task,
				(const char*	)"i2c_slave_task",
				(uint16_t		)I2CSLAVE_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)I2CSLAVE_TASK_PRIO,
				(TaskHandle_t*	)&I2CslaveTask_Handler);

	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL();
}



