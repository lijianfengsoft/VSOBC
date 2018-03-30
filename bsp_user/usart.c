#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "usart.h"
#include <sys/stat.h>
#include "stdint.h"
#include "stdio.h"

xQueueHandle  console_queue;
struct USART1_TypeDefStruct Console_Usart;



/**************************************************
 *  @Function name:串口数据的初始化
 *  @Author:LiJianfeng
 *  @data:2017/12/22 20:31
 *  @Inparam:无
 *  @Outparam:无
 *  @brief:初始化串口数据
 *************************************************/
void usart_confing(struct USART1_TypeDefStruct *pUSART)
{
	pUSART->USART_Read_Head = 0;
	pUSART->USART_Read_Tail = 0;
	pUSART->USART_Write_Head = 0;
	pUSART->USART_Write_Tail = 0;
	memset(pUSART->USART_Write_buf, 0, sizeof(pUSART->USART_Write_buf));
	memset(pUSART->USART_Read_buf, 0, sizeof(pUSART->USART_Read_buf));
	console_queue = xQueueCreate(CONSOLE_LENGTH, 1);
}

/**************************************************
 *  @Function name:串口1初始化
 *  @Author:LiJianfeng
 *  @data:2017/12/22 20:30
 *  @Inparam:波特率
 *  @Outparam:无
 *  @brief:USART1 PB6 PB7 开启接收中断
 *************************************************/
void usart_init(uint32_t bound) {

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

														 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); 

															  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
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
	USART_Init(USART1, &USART_InitStructure); 

	USART_Cmd(USART1, ENABLE);  

	USART_ClearFlag(USART1, USART_FLAG_TC);//清楚发送完成标志（CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去\
										   如下语句解决第1个字节无法正确发送出去的问题）

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
												  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                        
	NVIC_Init(&NVIC_InitStructure);	  

	usart_confing(&Console_Usart);
}
/**************************************************
 *  @Function name:串口中断函数（封装一层传入串口结构体）
 *  @Author:LiJianfeng
 *  @data:2017/12/22 20:28
 *  @Inparam:无
 *  @Outparam:无
 *  @brief:串口中断服务函数
 *************************************************/
void USART1_IRQHandler(void)
{
	USART1_IRQ(&Console_Usart);
}

void USART1_IRQ(struct USART1_TypeDefStruct *pUSART)
{
	uint8_t _rxData;    
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)      
	{
		_rxData = (uint8_t)(USART_ReceiveData(USART1) & 0xff);	 
		pUSART->USART_Read_buf[pUSART->USART_Read_Tail] = _rxData;
		Console_ReceivedByte(_rxData & 0xff);
		if (++(pUSART->USART_Read_Tail) == MAX_USART_BUFFSIZE)
			pUSART->USART_Read_Tail = 0;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET)
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if (pUSART->USART_Write_Head != pUSART->USART_Write_Tail)
		{
			USART_SendData(USART1, pUSART->USART_Write_buf[pUSART->USART_Write_Head]);
			if (++(pUSART->USART_Write_Head) == MAX_USART_BUFFSIZE)
				pUSART->USART_Write_Head = 0;
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
	}
	if (USART_GetITStatus(USART1, USART_IT_NE) != RESET)
		USART_ClearFlag(USART1, USART_FLAG_NE);
	if (USART_GetITStatus(USART1, USART_IT_FE) != RESET)
		USART_ClearFlag(USART1, USART_FLAG_FE);
	if (USART_GetITStatus(USART1, USART_IT_PE) != RESET)
		USART_ClearFlag(USART1, USART_FLAG_PE);	
}


/**************************************************
 *  @Function name:Console_ReceivedByte
 *  @Author:LiJianfeng
 *  @data:2017/12/22 20:05
 *  @Inparam:接收到的字节
 *  @Outparam:无
 *  @brief:串口接收到的数据送到处理函数（放在串口队列中）
 *************************************************/
void Console_ReceivedByte(uint8_t R_data)
{
	portBASE_TYPE xTaskWoken = pdFALSE;
	xQueueSendToBackFromISR(console_queue, &R_data, &xTaskWoken);
}

/**************************************************
 *  @Function name:Usart_read
 *  @Author:LiJianfeng
 *  @data:2017/12/22 21:02
 *  @Inparam:串口结构体
 *  @Outparam:从缓冲区读取的字节
 *  @brief:从串口缓冲区读取一个字节
 *************************************************/
uint8_t Usart_read(struct USART1_TypeDefStruct *pUSART)
{
	uint8_t readdata;
	readdata = pUSART->USART_Read_buf[pUSART->USART_Read_Head];
	if (++(pUSART->USART_Read_Head) == MAX_USART_BUFFSIZE)
		pUSART->USART_Read_Head = 0;
	return readdata;
}

/**************************************************
 *  @Function name:Usart_write
 *  @Author:LiJianfeng
 *  @data:2017/12/22 22:13
 *  @Inparam:串口结构体，待输入数据
 *  @Outparam:无
 *  @brief:向串口1输入1个字符
 *************************************************/
void Usart_write(struct USART1_TypeDefStruct *pUSART, uint8_t senddata)
{
	if (pUSART->USART_Write_Head==pUSART->USART_Write_Tail)
	{
		USART_SendData(USART1, senddata);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		return;
	}
	pUSART->USART_Write_buf[pUSART->USART_Write_Tail] = senddata;
	if (++(pUSART->USART_Read_Tail) == MAX_USART_BUFFSIZE)
		pUSART->USART_Write_Tail = 0;
}

/**************************************************
 *  @Function name:Usart_writes
 *  @Author:LiJianfeng
 *  @data:2017/12/22 22:19
 *  @Inparam:串口结构体，输入字符串
 *  @Outparam:无
 *  @brief:向串口输入字符串数据
 *************************************************/
void Usart_writes(struct USART1_TypeDefStruct *pUSART, uint8_t *sendbuff)
{
	uint8_t kc = 0;
	while (1)
	{
		if((pUSART->USART_Write_buf[pUSART->USART_Read_Tail]=sendbuff[kc++])=='\0')
			break;
		if (++(pUSART->USART_Write_Tail) == MAX_USART_BUFFSIZE)
			pUSART->USART_Write_Tail = 0;
	}
	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
}


/**************************************************
 *  @Function name:串口重定向
 *  @Author:LiJianfeng
 *  @data:2017/12/22 20:01
 *  @Inparam:
 *  @Outparam:
 *  @brief:在VS下使用以下函数进行printf的重定向
 fflush(stdout)刷新标准输出缓冲区，把输出缓冲区里的东西打印到标准输出设备上
 printf("。。。。。。。。。。。");后面加fflush(stdout)；可提高打印效率,
 现在解决方法是后面加\n刷新标志
 _sbrk dynamically increases the heap size. The heap starts right after the “end” symbol defined by the linker and is extended on demand until a collision with the stack pointer is detected.
 _fstat/_isatty are used to tell the C library that stdout is a console and not a file and should not be buffered excessively.
 _read/_write are called by the C library to actually transfer the data.
 *************************************************/

int _fstat(int fd, struct stat *pStat)
{
	pStat->st_mode = S_IFCHR;
	return 0;
}

int _close(int a)
{
	return -1;
}

int _write(int fd, char *pBuffer, int size)
{
	for (int i = 0; i < size; i++)
	{
		while (!(USART1->SR & USART_SR_TXE));
		USART_SendData(USART1, pBuffer[i]);
	}
	return size;
}

int _isatty(int fd)
{
	return 1;
}

int _lseek(int a, int b, int c)
{
	return -1;
}

int _read(int fd, char *pBuffer, int size)
{
	for (int i = 0; i < size; i++)
	{
		while ((USART1->SR & USART_SR_RXNE) == 0)
		{
		}

		pBuffer[i] = USART_ReceiveData(USART1);
	}
	return size;
}

caddr_t _sbrk(int increment)
{
	extern char end __asm__("end");
	register char *pStack __asm__("sp");

	static char *s_pHeapEnd;

	if (!s_pHeapEnd)
		s_pHeapEnd = &end;

	if (s_pHeapEnd + increment > pStack)
		return (caddr_t)-1;

	char *pOldHeapEnd = s_pHeapEnd;
	s_pHeapEnd += increment;
	return (caddr_t)pOldHeapEnd;
}

char * strdup(const char *s)
{
	size_t len = strlen(s) + 1;
	void *new = (void *)pvPortMalloc(len);
	if (new == NULL)
		return NULL;
	return (char *)memcpy(new, s, len);
}

size_t  strnlen(const char *str, size_t maxsize)
{
	size_t n;

	/* Note that we do not check if s == NULL, because we do not
	* return errno_t...
	*/

	for (n = 0; n < maxsize && *str; n++, str++)
		;

	return n;
}