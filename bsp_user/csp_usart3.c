#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"

#include "csp_debug.h"
#include "csp_usart.h"
#include "usart.h"

#define MAX_RECEIVE 235

static xQueueHandle USART_REC;
static usart_callback_t usart_callback = NULL;
static void usart3_rx_task(void *vptr_args);

void csp_usart3_init(uint32_t bound)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);

	USART_REC = xQueueCreate(MAX_RECEIVE, sizeof(uint8_t));
	if (USART_REC == NULL)
		return;

	portBASE_TYPE ret = xTaskCreate(usart3_rx_task, "usart3_rx_task", configMINIMAL_STACK_SIZE * 10, NULL, tskIDLE_PRIORITY + 6, NULL);
	if (ret != pdTRUE)
		csp_log_error("Failed to start usart router task\r\n");

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void usart3_putc(char c) {
	/* 发送一个字节数据到USART1 */
	USART_SendData(USART3, (uint8_t)c);

	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void usart_set_callback(usart_callback_t callback) {
	usart_callback = callback;
}

void usart_insert(char c, void * pxTaskWoken) {
	printf("%c", c);
}

static void usart3_rx_task(void *vptr_args)
{
	unsigned int length, count;
	static uint8_t * cbuf = NULL;
	uint8_t *ptr = NULL;

	if ((cbuf = (uint8_t *)pvPortMalloc(1000)) == NULL)
	{
		printf("malloc failed\r\n");
	}

	while (1) {

		length = uxQueueMessagesWaiting(USART_REC);
		ptr = cbuf;
		count = length;
		//printf("length is:%u\r\n",length);
		while (count--)
		{
			xQueueReceive(USART_REC, ptr++, 0);
			//printf( "%c\r\n", *ptr ); 
		}
		if (usart_callback&&length)
			//printf("excute call_back,%d  //\r\n",length);
			usart_callback(cbuf, (unsigned int)length, (void *)NULL);

		//	my_usart_rx(cbuf,length,NULL);

		vTaskDelay(100);
	}
}

void USART3_IRQHandler(void)
{
	uint8_t ch;

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{

		ch = USART_ReceiveData(USART3);
		//printf( "%c", ch );    //将接受到的数据直接返回打印
		xQueueSendFromISR(USART_REC, &ch, 0);
	}
}
