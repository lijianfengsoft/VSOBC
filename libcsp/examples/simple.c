/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


/******************************************
* 文件名:主函数
* 作者：王李
* 日期：2016-11-24
* 简介：XXXXXXXXXXXXXXXXX
*******************************************
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <csp/csp.h>

/* Using un-exported header file.
* This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>


//头文件
//?μí3í·???t
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

//stm32 头文件
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "bsp_usart.h"
#include "console.h"
#include "command.h"
#include "System.h"
#include "misc.h"
#include "led.h"


//指令系统包含
#include "bsp_usart.h"
#include "console.h"
#include "command.h"
//add by wl

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <csp/csp.h>

/* Using un-exported header file.
 * This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>

/** Example defines */
#define MY_ADDRESS  1			// Address of local CSP node
#define MY_PORT		10			// Port to send test traffic to

CSP_DEFINE_TASK(task_server) {



	/* Create socket without any socket options */
	csp_socket_t *sock = csp_socket(CSP_SO_NONE);

	/* Bind all ports to socket */
	csp_bind(sock, CSP_ANY);

	/* Create 10 connections backlog queue */
	csp_listen(sock, 10);

	/* Pointer to current connection and packet */
	csp_conn_t *conn;
	csp_packet_t *packet;

	/* Process incoming connections */
	while (1) {

		/* Wait for connection, 10000 ms timeout */
		if ((conn = csp_accept(sock, 10000)) == NULL)
			continue;

		/* Read packets. Timout is 100 ms */
		while ((packet = csp_read(conn, 100)) != NULL) {
			switch (csp_conn_dport(conn)) {
			case MY_PORT:
				/* Process packet here */
				printf("Packet received on MY_PORT: %s\r\n", (char *) packet->data);
				csp_buffer_free(packet);
				break;

			default:
				/* Let the service handler reply pings, buffer use, etc. */
				csp_service_handler(conn, packet);
				break;
			}
		}

		/* Close current connection, and handle next */
		csp_close(conn);

	}

	return CSP_TASK_RETURN;

}

CSP_DEFINE_TASK(task_client) {

	csp_packet_t * packet;
	csp_conn_t * conn;

	while (1) {

		/**
		 * Try ping
		 */

		csp_sleep_ms(1000);

		int result = csp_ping(MY_ADDRESS, 100, 100, CSP_O_NONE);
		printf("Ping result %d [ms]\r\n", result);

		csp_sleep_ms(1000);

		/**
		 * Try data packet to server
		 */

		/* Get packet buffer for data */
		packet = csp_buffer_get(100);
		if (packet == NULL) {
			/* Could not get buffer element */
			printf("Failed to get buffer element\n");
			return CSP_TASK_RETURN;
		}

		/* Connect to host HOST, port PORT with regular UDP-like protocol and 1000 ms timeout */
		conn = csp_connect(CSP_PRIO_NORM, MY_ADDRESS, MY_PORT, 1000, CSP_O_NONE);
		if (conn == NULL) {
			/* Connect failed */
			printf("Connection failed\n");
			/* Remember to free packet buffer */
			csp_buffer_free(packet);
			return CSP_TASK_RETURN;
		}

		/* Copy dummy data to packet */
		char *msg = "Hello World";
		strcpy((char *) packet->data, msg);

		/* Set packet length */
		packet->length = strlen(msg);

		/* Send packet */
		if (!csp_send(conn, packet, 1000)) {
			/* Send failed */
			printf("Send failed\n");
			csp_buffer_free(packet);
		}

		/* Close connection */
		csp_close(conn);

	}

	return CSP_TASK_RETURN;
}


void Vtask1(void *p);
void Vtask2(void *p);


int main(int argc, char * argv[]) {

    
	
	/* 嵌套向量中断控制器组选择 */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//璁剧疆鏃堕挓,168Mhz
	Stm32_Clock_Init(336,8,2,7);

	Console_Usart_init(115200);

	LED_GPIO_Config();

	csp_log_error("....testing ....\r\n");//mark by wl


	//{
	//	portBASE_TYPE ret   =  xTaskCreate(Vtask1, "task1", 500, NULL, tskIDLE_PRIORITY+ 2, NULL);
	//	if(ret!=pdTRUE)
	//	{
	//		csp_log_error("Failed to start router task\n");//mark by wl
	//		return CSP_ERR_NOMEM;
	//	}
	//}
	//
	//{

	//	portBASE_TYPE ret   =  xTaskCreate(Vtask2, "task2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+ 2, NULL);
	//	if(ret!=pdTRUE)
	//	{
	//		csp_log_error("Failed to start router task\n");//mark by wl
	//		return CSP_ERR_NOMEM;
	//	}
	//}
	//

	//vTaskStartScheduler();

	//while(1){
	//}

	/**
	 * Initialise CSP,
	 * No physical interfaces are initialised in this example,
	 * so only the loopback interface is registered.
	 */
	/* Init buffer system with 10 packets of maximum 300 bytes each */

	printf("Initialising CSP\r\n");

	csp_buffer_init(5,300);


	/* Init CSP with address MY_ADDRESS */
	csp_init(MY_ADDRESS);


	/* Start router task with 500 word stack, OS task priority 1 */
	csp_route_start_task(500, 1);


	/* Enable debug output from CSP */
	if ((argc > 1) && (strcmp(argv[1], "-v") == 0)) {
		printf("Debug enabed\r\n");
		csp_debug_toggle_level(3);
		csp_debug_toggle_level(4);

		printf("Conn table\r\n");
		csp_conn_print_table();

		printf("Route table\r\n");
		csp_route_print_table();

		printf("Interfaces\r\n");
		csp_route_print_interfaces();

	}

	/**
	 * Initialise example threads, using pthreads.
	 */

	/* Server */
	printf("Starting Server task\r\n");
	csp_thread_handle_t handle_server;
	csp_thread_create(task_server, (signed char *) "SERVER", 1000, NULL, 0, &handle_server);

	/* Client */
	printf("Starting Client task\r\n");
	csp_thread_handle_t handle_client;
	csp_thread_create(task_client, (signed char *) "SERVER", 1000, NULL, 0, &handle_client);

	/* Wait for execution to end (ctrl+c) */
	while(1) {
		csp_sleep_ms(100000);
	}

	return 0;

}


void Vtask1(void *p)
{	

	while (1)
	{

		GPIO_ResetBits(GPIOC, GPIO_Pin_11);
		vTaskDelay(500);

		GPIO_SetBits(GPIOC, GPIO_Pin_11);
		vTaskDelay(500);
	}
}


void Vtask2(void *p)
{	

	while (1)
	{
		printf("testing...1\r\n");
		vTaskDelay(500);
		printf("testing...2\r\n");
		vTaskDelay(500);
	}
}


