/***************************************************************************
*  功能：
*  版本：
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/24 0:18 
*****************************************************************************/                                                    

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "stdio.h"
#include "usart.h"
#include "bsp_usart.h"
#include "bsp_norflash.h"
#include "ff.h"
#include "sdio_sdcard.h"

#include "string.h"

#define RESERVED_MASK   (uint32_t)0x0F7D0F7D

static uint32_t DataOffset = 1; //用来表征发起了几次DMA传输，还有用来计算需要设置的存储器地址，初始值为1

								
uint8_t Picturedata[500 * 1024] __attribute__((section(".exbss"), used));

uint8_t sendcmd[6];

uint32_t Picture_Rxlen;

void bsp_usart_init(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	NVIC_InitTypeDef    NVIC_InitStructure;
	USART_InitTypeDef   USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	/* Configure USART Tx and Rx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);
	/* NVIC configuration */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	/* USART interrupt configuration */
	USART_ITConfig(USART3, USART_IT_TC, DISABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);				//检测到空闲线路时，该位由硬件置 1。如果 USART_CR1 寄存器中 IDLEIE = 1，则会生成中断。
	USART_Cmd(USART3, ENABLE);
	/* 串口发送DMA使能 */
	USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
}

void USART3_DMA_Config(void)
{
	DMA_InitTypeDef     DMA_InitStructure;
	NVIC_InitTypeDef    NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);						//DMA1时钟使能

	DMA_DeInit(DMA1_Stream1);
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE) {}						//等待DMA可配置

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;								//通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);			//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Picturedata;				//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;						//外设到内存模式 
	DMA_InitStructure.DMA_BufferSize = 65535;									//数据传输量(范围为1-65535) 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//内存地址增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//外设数据大小1byte
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				//内存数据大小1byte
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								//使用普通模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							//优先级高
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						//不开fifo
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;   
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					//存储器突发单次传输 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			//外设突发单次传输
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);									//初始化dma1 stream1

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);

	DMA_DeInit(DMA1_Stream3);
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE) {}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);			
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sendcmd;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					
	DMA_InitStructure.DMA_BufferSize = 6;							 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			   	 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;						 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;			 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			 
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);                          //初始化dma1 stream3

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA1_Stream1, ENABLE);
}


/* 串口空闲中断 */
void USART3_IRQHandler(void)
{
	uint8_t ReceivedData;
	static BaseType_t TaskWoken = pdFALSE;
	uint16_t CleanUpRegist;

	//if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	//{
	//	USART_ClearFlag(USART3, USART_FLAG_ORE);
	//	ReceivedData = USART_ReceiveData(USART3);
	//	printf("Receive overflow error!\n\r");
	//}
	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		CleanUpRegist = USART3->SR;					//该位由软件序列清零（读入 USART_SR 寄存器，然后读入 USART_DR 寄存器）
		CleanUpRegist = USART3->DR;

		USART_ClearFlag(USART3, USART_FLAG_IDLE);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);

		Picture_Rxlen = 65535 * DataOffset -
			DMA_GetCurrDataCounter(DMA1_Stream1);
		printf("Data_Len = %u\n\r", Picture_Rxlen);
		printf("Address = %p\n\r", Picturedata);

		DataOffset = 0;

		DMA_Cmd(DMA1_Stream1, DISABLE);
	}
	portYIELD_FROM_ISR(TaskWoken);
}

/* DMA传输完成中断设置，因为DMA每次最大传输65535个字
节数据，为了实现连续串口接收超过65535字节数据 */
void DMA1_Stream1_IRQHandler(void)
{
	if ((DMA1_Stream1->CR & (uint32_t)DMA_SxCR_EN) != SET)
	{
		DMA1->LIFCR = (uint32_t)(DMA_FLAG_TCIF1 & RESERVED_MASK);  //清传输完成标志（其实清除了所有标志）

		if (!DataOffset)
			DMA1_Stream1->NDTR = 0;

		DMA1_Stream1->M0AR = (uint32_t)&Picturedata[65535*DataOffset -DMA1_Stream1->NDTR];  //设置存储器地址
		DMA1_Stream1->NDTR = (uint16_t)65535;//编程DMA接收字节数
		DMA1_Stream1->CR |= DMA_SxCR_EN;    //使能 串口3的DMA接收
		DataOffset++;
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	if (DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) != RESET)
	{
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	}
}

