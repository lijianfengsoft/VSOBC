#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "console.h"
#include "command.h"
#include "bsp_i2c_task.h"
#include "csp_thread.h"
#include "csp_usart.h"


#define START_TASK_PRIO  1					//开始任务优先级
#define START_STK_SIZE  128					//开始任务堆栈
TaskHandle_t StartTask_Handler;             //开始任务句柄
void start_task(void *pvParameters);		//开始任务实体函数声明

#define DEBUG_TASK_PRIO  4					//调试任务优先级
#define DEBUG_STK_SIZE  130*4				//调试任务堆栈
TaskHandle_t DebugTask_Handler;             //调试任务句柄

#define I2CSLAVE_TASK_PRIO  2					//I2C从机任务优先级
#define I2CSLAVE_STK_SIZE  130*4				//I2C从机任务堆栈
TaskHandle_t I2CslaveTask_Handler;             //I2C从机任务句柄

#define I2CMASTER_TASK_PRIO  2					//I2C主机任务优先级
#define I2CMASTER_STK_SIZE  130*4				//I2C主机任务堆栈
TaskHandle_t I2CmasterTask_Handler;             //I2C主机任务句柄

#define CSP_TASK_CLINET_PRIO 3
#define CLINER_STK_SIZE		130*10
csp_thread_handle_t	handle_client;

#define CSP_TASK_CLINET_PRIO 3
#define CLINER_STK_SIZE		130*10
csp_thread_handle_t	handle_server;

CSP_DEFINE_TASK(task_server) {
	int running = 1;
	csp_socket_t *socket = csp_socket(CSP_SO_NONE);
	csp_conn_t *conn;
	csp_packet_t *packet;
	csp_packet_t *response;

	response = csp_buffer_get(sizeof(csp_packet_t) + 2);
	if (response == NULL) {
		return CSP_TASK_RETURN;
	}
	response->data[0] = 'O';
	response->data[1] = 'K';
	response->length = 2;

	csp_bind(socket, CSP_ANY);
	csp_listen(socket, 5);

	printf("Server task started\r\n");

	while (running) {
		if ((conn = csp_accept(socket, 10000)) == NULL) {
			continue;
		}

		while ((packet = csp_read(conn, 100)) != NULL) {
			switch (csp_conn_dport(conn)) {
			case 11:
				if (packet->data[0] == 'q')
					running = 0;
				csp_buffer_free(packet);
				csp_send(conn, response, 1000);
				break;
			default:
				csp_service_handler(conn, packet);
				break;
			}
		}

		csp_close(conn);
	}

	csp_buffer_free(response);

	return CSP_TASK_RETURN;
}

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
		
		while (1);

		return 0;
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

	csp_thread_create((TaskFunction_t			)task_client, 
					  (const signed char * const)"CLINET", 
					  (unsigned short			)CLINER_STK_SIZE, 
	  				  (void *					)NULL, 
					  (unsigned int				)CSP_TASK_CLINET_PRIO, 
					  (csp_thread_handle_t *	)&handle_client);

	csp_thread_create((TaskFunction_t			)task_server,
					 (const signed char * const)"CLINET",
					 (unsigned short			)CLINER_STK_SIZE,
					 (void *					)NULL,
					 (unsigned int				)CSP_TASK_CLINET_PRIO,
					 (csp_thread_handle_t *		)&handle_server);
	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL();
}



