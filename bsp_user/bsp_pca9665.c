/***************************************************************************
*  功能：配置与PCA9665相关的配置
*  版本：V 1.0
*  迭代：此配置未实现PCA9665所有的情况，实现了主发和主收和从收的缓冲模式，对于字节模式和从发模式
		未实现，若以后用到可参考gaomspace的例程来进行对应的添加
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/15 20:56 
*****************************************************************************/                                                    

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_fsmc.h"
#include "misc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#include "driver_debug.h"
#include "error.h"
#include "bsp_pca9665.h"
#include "delay.h"
#include "string.h"

static int pca9665_init_registers(int handle);
static inline void pca9665_soft_reset(int handler);
static int pca9665_init(int handle, uint8_t * base_addr, uint8_t addr, uint16_t speed);
static inline void pca9665_write_reg(int handler, uint8_t reg, uint8_t val);
static inline uint8_t pca9665_read_reg(int handler, uint8_t reg);
static void pca9665_try_tx_from_isr(int handle, portBASE_TYPE * pxTaskWoken);
static inline void pca9665_write_data(int handler, uint8_t* src, int len);
static inline void pca9665_read_data_to_buffer(int handle);
static inline void pca9665_read_data(int handler, uint8_t* dst, int len);


const int pca9665_device_count = 2;

SemaphoreHandle_t i2c_lock = NULL;

pca9665_device_object_t device[2] =
{
	{
		.base = (uint8_t *)FSMC_Bank1_SRAM1_ADDR,
		.is_initialised = 0,
		.is_busy = 0,
		.mode = DEVICE_MODE_M_T,
		.callback = NULL,
	},
	{
		.base = (uint8_t *)FSMC_Bank1_SRAM4_ADDR,
		.is_initialised = 0,
		.is_busy = 0,
		.mode = DEVICE_MODE_M_T,
		.callback = NULL,
	},
};

void PCA9665_IO_Init(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  p;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
							RCC_AHB1Periph_GPIOG, ENABLE);

	/* Enable FSMC clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	/*-- GPIOs Configuration -----------------------------------------------------*/
	/*
	+-------------------+--------------------+------------------+------------------+
	+                       SRAM pins assignment                                   +
	+-------------------+--------------------+------------------+------------------+
	| PD0  <-> FSMC_D2  | PE0  <-> FSMC_NBL0 | PF0  <-> FSMC_A0 | PG0 <-> FSMC_A10 |
	| PD1  <-> FSMC_D3  | PE1  <-> FSMC_NBL1 | PF1  <-> FSMC_A1 | PG1 <-> FSMC_A11 |
	| PD4  <-> FSMC_NOE | PE3  <-> FSMC_A19  | PF2  <-> FSMC_A2 | PG2 <-> FSMC_A12 |
	| PD5  <-> FSMC_NWE | PE4  <-> FSMC_A20  | PF3  <-> FSMC_A3 | PG3 <-> FSMC_A13 |
	| PD8  <-> FSMC_D13 | PE7  <-> FSMC_D4   | PF4  <-> FSMC_A4 | PG4 <-> FSMC_A14 |
	| PD9  <-> FSMC_D14 | PE8  <-> FSMC_D5   | PF5  <-> FSMC_A5 | PG5 <-> FSMC_A15 |
	| PD10 <-> FSMC_D15 | PE9  <-> FSMC_D6   | PF12 <-> FSMC_A6 | PD7 <-> FSMC_NE1 |
	| PD11 <-> FSMC_A16 | PE10 <-> FSMC_D7   | PF13 <-> FSMC_A7 | PG9 <-> FSMC_NE2 |
	| PD12 <-> FSMC_A17 | PE11 <-> FSMC_D8   | PF14 <-> FSMC_A8 | PG10 <-> FSMC_NE3|
	| PD13 <-> FSMC_A18 | PE12 <-> FSMC_D9   | PF15 <-> FSMC_A9 | PG12 <-> FSMC_NE4|
	| PD14 <-> FSMC_D0  | PE13 <-> FSMC_D10  |------------------+------------------+
	| PD15 <-> FSMC_D1  | PE14 <-> FSMC_D11  |
	|                   | PE15 <-> FSMC_D12  |
	+-------------------+--------------------+
	*/
	/* GPIOD configuration */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 |
									GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
									GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* GPIOE configuration */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 |
									GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
									GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
									GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);


	/* GPIOF configuration */
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
									GPIO_Pin_4 | GPIO_Pin_5 |
									GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;		
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* GPIOG configuration */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
									GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/*-- FSMC Configuration ------------------------------------------------------*/
	p.FSMC_AddressSetupTime = 5;
	p.FSMC_AddressHoldTime = 3;
	p.FSMC_DataSetupTime = 6;
	p.FSMC_BusTurnAroundDuration = 0;
	p.FSMC_CLKDivision = 0;
	p.FSMC_DataLatency = 0;
	p.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/*!< Enable FSMC Bank1_SRAM2 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
	/*!< Enable FSMC Bank1_SRAM2 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

int i2c_init(int handle, int mode, uint8_t addr, uint16_t speed, int queue_len_tx, int queue_len_rx, i2c_callback_t callback) 
{

	/* Error checking */
	if (handle >= pca9665_device_count)
		return E_NO_DEVICE;

	if (device[handle].is_initialised)
		return E_NO_ERR;

	if (queue_len_tx <= 0)
		return E_INVALID_PARAM;

	if (mode != I2C_MASTER) 
	{
		driver_debug(DEBUG_I2C, "PCA9665 driver does not support slave mode operation\r\n");
		return E_INVALID_PARAM;
	}

	if (i2c_lock == NULL)
		i2c_lock = xSemaphoreCreateMutex();
	if (i2c_lock == NULL)
		return E_NO_BUFFER;

	/* 主发队列 */
	if (device[handle].tx.queue == NULL) 
	{
		device[handle].tx.queue = xQueueCreate(queue_len_tx, sizeof(i2c_frame_t *));
	}

	/* 主收队列 */
	if ((device[handle].rx.queue == NULL) && (queue_len_rx > 0)) 
	{
		device[handle].rx.queue = xQueueCreate(queue_len_rx, sizeof(i2c_frame_t *));
	}

	/* Callback */
	if (callback != NULL) 
	{
		device[handle].callback = callback;
	}

	/* Initialise device */
	int result = pca9665_init(handle, device[handle].base, addr, speed);
	if (result == E_NO_ERR) 
	{
		device[handle].is_initialised = 1;
	}
	else
	{
		driver_debug(DEBUG_I2C, "Failed to initialise PCA9665 driver\r\n");
	}

	return result;
}

static int pca9665_init(int handle, uint8_t * base_addr, uint8_t addr, uint16_t speed) 
{
	/* Remember chip memory address, and I2C node address */
	device[handle].base = base_addr;
	device[handle].slave_addr = addr;
	device[handle].speed = speed;

	/* Call HW init, sets up registers */
	if (pca9665_init_registers(handle) < 0)
		return E_GARBLED_BUFFER;

	/* Enable */
	pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA);

	return E_NO_ERR;
}

static int pca9665_init_registers(int handle)
{
		/* Soft reset the device */
		pca9665_soft_reset(handle);

		int retry = 10;
		while (retry--) 
		{
			if ((pca9665_read_reg(handle, I2CCON) & 0x40) == 0)
				break;
			pca9665_soft_reset(handle);
			delay_us(1000);
		}
		/* Setup: I2C Addr */
		pca9665_write_reg(handle, I2CADR, device[handle].slave_addr << 1);
		if (pca9665_read_reg(handle, I2CADR) != (device[handle].slave_addr << 1))
			return -1;

		/* Setup: I2C mode */
		pca9665_write_reg(handle, I2CMODE, 0x01);

		/* Setup: I2C Timeout */
		pca9665_write_reg(handle, I2CTO, 0xFF);

		/* Setup: Speed
		*                               1
		* f SCL = -------------------------------------
		*          Tosc (I2CSCLL + I2CSCLH) + Tr + Tf 
		* */
		double Tr = 300e-9;		// Estimated Trise in s
		double Tf = 300e-9;		// Estimated Tfall in s
		double Tosc = 30e-9;	// Tosc in s
		double period = 1 / (Tosc * device[handle].speed * 1000) - (Tr + Tf) / Tosc;

		pca9665_write_reg(handle, I2CSCLL, (uint8_t)(period * 0.68));
		pca9665_write_reg(handle, I2CSCLH, (uint8_t)(period * 0.32));

		return 0;
}

static inline void pca9665_soft_reset(int handler) 
{
	driver_debug(DEBUG_I2C, "RESET\r\n");

	*(uint8_t *)(device[handler].base + INDPTR) = I2CPRESET - 4;
	*(uint8_t *)(device[handler].base + INDIRECT) = 0xA5;
	*(uint8_t *)(device[handler].base + INDIRECT) = 0x5A;
}

static inline void pca9665_write_reg(int handler, uint8_t reg, uint8_t val) 
{
	//driver_debug(DEBUG_I2C, "Write reg %x, %x\r\n", reg, val);

	if (reg < 0x04) 
	{
		*(uint8_t *)(device[handler].base + reg) = val;
	}
	else
	{
		*(uint8_t *)(device[handler].base + INDPTR) = reg - 4;
		*(uint8_t *)(device[handler].base + INDIRECT) = val;
	}

}

static inline uint8_t pca9665_read_reg(int handler, uint8_t reg)
{
	//driver_debug(DEBUG_I2C, "Read reg %x at %p\r\n", reg, device[handler].base + reg);

	if (reg < 0x04)
		return *(uint8_t *)(device[handler].base + reg);
	else 
	{
		*(uint8_t *)(device[handler].base + INDPTR) = reg - 4;
		return *(uint8_t *)(device[handler].base + INDIRECT);
	}
}

void pca9665_isr_init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //I2C0--PC0, I2C1--PC1
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line1);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void)
{
	uint32_t status_value;
	static portBASE_TYPE TaskWokenExit0;

	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);

		TaskWokenExit0 = pdFALSE;

		status_value = taskENTER_CRITICAL_FROM_ISR();

		pca9665_dsr(&TaskWokenExit0);

		taskEXIT_CRITICAL_FROM_ISR(status_value);

		portYIELD_FROM_ISR(TaskWokenExit0);
	}
}

void EXTI1_IRQHandler(void) 
{
	uint32_t status_value;
	static portBASE_TYPE TaskWokenExit1;

	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);

		TaskWokenExit1 = pdFALSE;

		status_value = taskENTER_CRITICAL_FROM_ISR();

		pca9665_dsr(&TaskWokenExit1);

		taskEXIT_CRITICAL_FROM_ISR(status_value);

		portYIELD_FROM_ISR(TaskWokenExit1);
	}
}

void __attribute__((noinline)) pca9665_dsr(portBASE_TYPE * task_woken)
{
	static int handle, len;
	static uint8_t state;
	static uint8_t dest;

	/* Loop through number of devices */
	for (handle = 0; handle < pca9665_device_count; handle++)
	{
		/* Check for interrupt flag in device status register */
		if (!(pca9665_read_reg(handle, I2CCON) & CON_SI))
			continue;

		/* We have an interrupt, read the status register */
		state = pca9665_read_reg(handle, I2CSTA);

		/* The I2C driver is one _big_ state-machine */
		driver_debug(DEBUG_I2C, "I2C ISR %u %x\n\r", handle, state);

		switch (state)
		{
			/**
			* MASTER IRQ's
			*/
		/* START: is the first ISR that appears for outgoing frames */
		case STA_M_REPEATED_START_SENDT:
		case STA_M_START_SENDT:
			/* Mark as busy, so start flag is not sent from task context while transmission is active */
			device[handle].is_busy = 1;

			/* If this is the beginning of a new frame, dequeue */
			if (device[handle].tx.frame == NULL && device[handle].rx.frame == NULL)
			{
				/* Try do dequeue element, if it fails, stop transmission */
				xQueueReceiveFromISR(device[handle].tx.queue, &device[handle].tx.frame, task_woken);
				if (device[handle].tx.frame == NULL) 
				{
					pca9665_try_tx_from_isr(handle, task_woken);
					break;
				}
				/* If TX len > 0, go for master transmit */
				if (device[handle].tx.frame->len) 
				{
					device[handle].mode = DEVICE_MODE_M_T;
					device[handle].tx.next_byte = 0;

					/* If TX len == 0 and RX len > 0, go for master receive */
				}
				else if (device[handle].tx.frame->len_rx)
				{
					device[handle].mode = DEVICE_MODE_M_R;
					device[handle].rx.frame = device[handle].tx.frame;
					device[handle].tx.frame = NULL;
					device[handle].rx.frame->len = device[handle].rx.frame->len_rx;
					device[handle].rx.next_byte = 0;
					/* Well, this should not happen */
				}
				else 
				{
					vPortFree(device[handle].tx.frame);
					device[handle].tx.frame = NULL;
					pca9665_try_tx_from_isr(handle, task_woken);
					break;
				}
			}
			/* If mode is master receiver then set the read-bit in the address field */
			if (device[handle].mode == DEVICE_MODE_M_R)
			{
				dest = (device[handle].rx.frame->dest << 1) | 0x01;
				device[handle].rx.next_byte = 0;

				/* Do first part of frame here */
				if (device[handle].rx.frame->len > PCA9665_MAX_BUF)
				{
					pca9665_write_reg(handle, I2CCOUNT, PCA9665_MAX_BUF);
				}
				else
				{
					pca9665_write_reg(handle, I2CCOUNT, device[handle].rx.frame->len | 0x80);
				}
				pca9665_write_data(handle, &dest, 1);
			}
			else     //If mode is master transmitter then set the write-bit in the address field
			{
				dest = device[handle].tx.frame->dest << 1 | 0x00;
				device[handle].tx.next_byte = 0;

				/* Do first part of frame here */
				if (device[handle].tx.frame->len + 1 > PCA9665_MAX_BUF)
				{
					pca9665_write_reg(handle, I2CCOUNT, PCA9665_MAX_BUF);
					pca9665_write_data(handle, &dest, 1);
					pca9665_write_data(handle, &device[handle].tx.frame->data[device[handle].tx.next_byte], PCA9665_MAX_BUF - 1);
					device[handle].tx.next_byte += PCA9665_MAX_BUF - 1;
				}
				else
				{
					pca9665_write_reg(handle, I2CCOUNT, device[handle].tx.frame->len + 1);
					pca9665_write_data(handle, &dest, 1);
					pca9665_write_data(handle, &device[handle].tx.frame->data[device[handle].tx.next_byte], device[handle].tx.frame->len);
					device[handle].tx.next_byte += device[handle].tx.frame->len;
				}
			}

			/* Let the hardware continue */
			pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA);
			break;

		/* WRITE ACK: A node is ready to be written to */
		case STA_M_SLAW_SENDT_ACKED:
		case STA_M_DATA_SENDT_ACKED:

			/* Safety first */
			if (device[handle].tx.frame == NULL)
				goto isr_error;

			/* Calculate remaining length */
			len = device[handle].tx.frame->len - device[handle].tx.next_byte;

			/* Transmit next chunk */
			if (len > 0)
			{
				if (len > PCA9665_MAX_BUF)
				{
					pca9665_write_reg(handle, I2CCOUNT, PCA9665_MAX_BUF);
					pca9665_write_data(handle, &device[handle].tx.frame->data[device[handle].tx.next_byte], PCA9665_MAX_BUF);
					device[handle].tx.next_byte += PCA9665_MAX_BUF;
				}
				else
				{
					pca9665_write_reg(handle, I2CCOUNT, len);
					pca9665_write_data(handle, &device[handle].tx.frame->data[device[handle].tx.next_byte], len);
					device[handle].tx.next_byte += len;
				}
				pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA);
				break;
			}
			/* Or, Change from master transmit, to master read if wanted */
			else if (device[handle].tx.frame->len_rx)
			{
				device[handle].mode = DEVICE_MODE_M_R;
				device[handle].rx.frame = device[handle].tx.frame;
				device[handle].tx.frame = NULL;
				device[handle].rx.frame->len = device[handle].rx.frame->len_rx;
				/* We need to send a repeated start now! */
				pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA | CON_STA);
				break;
			}
			else
			{
				vPortFree(device[handle].tx.frame);
				device[handle].tx.frame = NULL;
				pca9665_try_tx_from_isr(handle, task_woken);
			}
			break;

		/* WRITE ERROR: A write has failed */
		case STA_M_SLAW_SENDT_NACKED:
		case STA_M_DATA_SENDT_LAST_NACKED:
			if (device[handle].tx.frame != NULL)
			{
				driver_debug(DEBUG_I2C, "I2C SLA+W NACK: 0x%02x\n\r", device[handle].tx.frame->dest);
				vPortFree(device[handle].tx.frame);
				device[handle].tx.frame = NULL;
			}
			pca9665_try_tx_from_isr(handle, task_woken);
			break;

		/* ARBITRATION LOST: Start condition failed */
		case STA_M_ARBITRATION_LOST:
			/* Restart transmission by resetting next_byte and preserving tx_frame pointer */
			device[handle].tx.next_byte = 0;
			pca9665_try_tx_from_isr(handle, task_woken);
			break;

		/* READ ACK: A node is ready to be read from */
		case STA_M_SLAR_SENT_ACKED:
		case STA_M_DATA_RECEIVED_ACKED:
		case STA_M_DATA_RECEIVED_NACKED:
			/* Safety first */
			if (device[handle].rx.frame == NULL)
				goto isr_error;

			if (device[handle].rx.queue == NULL)
				goto isr_error;

			pca9665_read_data_to_buffer(handle);
			int remaining = device[handle].rx.frame->len - device[handle].rx.next_byte;
			driver_debug(DEBUG_I2C, "RX: Remaining %u\r\n", remaining);

			/* If no more to receive */
			if (remaining == 0)
			{
				if (xQueueSendToBackFromISR(device[handle].rx.queue, &device[handle].rx.frame, task_woken) == pdFALSE)
				{
					driver_debug(DEBUG_I2C, "I2C rx queue full - freeing\n\r");
					vPortFree(device[handle].rx.frame);
				}
				device[handle].rx.frame = NULL;
				pca9665_try_tx_from_isr(handle, task_woken);
				break;
			}

			/* If more than a full PCA9665 buffer remains */
			if (remaining > PCA9665_MAX_BUF) 
			{
				pca9665_write_reg(handle, I2CCOUNT, PCA9665_MAX_BUF);		
			}
			else /* Otherwise, this is the last bit, set NACK on final slave read */
			{
				pca9665_write_reg(handle, I2CCOUNT, remaining | 0x80);
			}

			pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA);
			break;

		/* READ ERROR: A read has failed */
		case STA_M_SLAR_SENT_NACKED:

			/* Safety first */
			if (device[handle].rx.frame == NULL)
				goto isr_error;

			driver_debug(DEBUG_I2C, "I2C SLA+R nacked\n\r");

			vPortFree(device[handle].rx.frame);
			device[handle].rx.frame = NULL;

			/* Start up again */
			pca9665_try_tx_from_isr(handle, task_woken);
			break;
		/**
		* SLAVE RECEIVER BUFFERED MODE
		*/
		/* START: Lost the arbitration and is addressed as a slave receiver */
		case STA_S_ARB_LOST_SLAW_RECEIVED:
		case STA_S_ARB_LOST_GC_RECEIVED:

			/* Preserve TX frame active, so the START flag will be re-set when the
			* reception is completed
			*/

			/* Deliberate Fallthrough */

		/* START: Addressed as a slave receiver */
		case STA_S_SLAW_RECEIVED_ACKED:
		case STA_S_GC_RECEIVED:

			device[handle].is_busy = 1;

			/* Check if RX frame was started */
			if (device[handle].rx.frame != NULL)
				goto isr_error;

			/* Allocate new frame */
			device[handle].rx.frame = (i2c_frame_t *)pvPortMalloc(I2C_MTU);
			if (device[handle].rx.frame == NULL)
				goto isr_error;

			device[handle].rx.next_byte = 0;
			device[handle].rx.frame->len = 0;
			device[handle].rx.frame->dest = device[handle].slave_addr;
			pca9665_write_reg(handle, I2CCOUNT, PCA9665_MAX_BUF);
			pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA);
			break;

		/* READ: Data received. */
		case STA_S_DATA_RECEIVED_SLA_ACKED:
		case STA_S_DATA_RECEIVED_GC_ACKED:
			/* Safety first */
			if (device[handle].rx.frame == NULL)
				goto isr_error;

			/* Receive data, if any */
			pca9665_read_data_to_buffer(handle);

			/* Limit incoming bytes */
			pca9665_write_reg(handle, I2CCOUNT, (device[handle].rx.next_byte + PCA9665_MAX_BUF > I2C_MTU) ? (I2C_MTU - device[handle].rx.next_byte) | 0x80 : PCA9665_MAX_BUF);

			pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA);
			break;

		/* STOP or NACK: No more data to receive */
		case STA_S_STOP_REP_RECEIVED:
		case STA_S_DATA_RECEIVED_SLA_NACKED:
		case STA_S_DATA_RECEIVED_GC_NACKED:
			/* Safety first */
			if (device[handle].rx.frame == NULL)
				goto isr_error;

			/* Receive data, if any */
			pca9665_read_data_to_buffer(handle);

			/* Queue up frame
			* Callback takes priority over RX queue
			* I2C master transaction temporarily disables the callback during master transactions in order to
			* ensure that the message is placed in the RX queue. */
			device[handle].rx.frame->len = device[handle].rx.next_byte;
			if (device[handle].callback != NULL)
			{
				device[handle].callback(device[handle].rx.frame, task_woken);
			}
			else if (device[handle].rx.queue != NULL)
			{
				if (xQueueSendToBackFromISR(device[handle].rx.queue, &device[handle].rx.frame, task_woken) == pdFALSE)
				{
					driver_debug(DEBUG_I2C, "I2C RX queue full\n\r");
					vPortFree(device[handle].rx.frame);
				}
			}
			else
			{
				vPortFree(device[handle].rx.frame);
			}
			/* The frame has been freed now */
			device[handle].rx.frame = NULL;

			/* Set back to master mode */
			pca9665_try_tx_from_isr(handle, task_woken);
			break;

		/**
		* Other IRQ's, typically indicates a hardware or protcol error
		* The IDLE status is considered an error if asserted at the same time as the Serial Interrupt flag
		*/
		case STA_IDLE:
		default:
		isr_error:
			i2c_error_count++;
			/* Soft reset the device */
			driver_debug(DEBUG_I2C, "I2C ERR 0x%02X\n\r", state);
			pca9665_init_registers(handle);

			/* Clean up RX */
			if (device[handle].rx.frame != NULL)
			{
				vPortFree(device[handle].rx.frame);
				device[handle].rx.frame = NULL;
			}
			/* Clean up TX */
			if (device[handle].tx.frame != NULL)
			{
				vPortFree(device[handle].tx.frame);
				device[handle].tx.frame = NULL;
			}
			/* Start up again */
			pca9665_try_tx_from_isr(handle, task_woken);
			break;
		}
	}
}

/**
* Pick up a transmission from scratch or continue an on-going transmission
* Context: ISR ONLY
* @param handle Handle to device
*/
static void pca9665_try_tx_from_isr(int handle, portBASE_TYPE * pxTaskWoken)
{
	uint8_t flags = 0;

	if (device[handle].is_initialised == 0)
		return;

	if (uxQueueMessagesWaitingFromISR(device[handle].tx.queue) > 0 || device[handle].tx.frame != NULL) 
	{
		device[handle].is_busy = 1;
		flags |= CON_STA;
	}
	else
	{
		device[handle].is_busy = 0;
		flags |= CON_STO;
	}

	/* Send the start/stop/restart condition */
	device[handle].mode = DEVICE_MODE_M_T;
	pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA | flags);
}

/**
* Write a block of data directly to data register
*
* @param handler Device handle
* @param src Source data pointer
* @param len number of bytes to write
* @param offset number of bytes to offset the write
*/
static inline void pca9665_write_data(int handler, uint8_t* src, int len) 
{
	while (len--)
		*(uint8_t *)(device[handler].base + I2CDAT) = *src++;
}

static inline void pca9665_read_data_to_buffer(int handle) 
{

	if (device[handle].rx.frame == NULL)
		return;

	/* Check number of bytes */
	int count = pca9665_read_reg(handle, I2CCOUNT) & 0x7f;
	if (count == 0)
		return;

	/* Check buffer allocation */
	if (count + device[handle].rx.next_byte > I2C_MTU)
		return;

	/* Copy data and increment next_byte counter */
	pca9665_read_data(handle, &device[handle].rx.frame->data[device[handle].rx.next_byte], count);
	device[handle].rx.next_byte += count;
	driver_debug(DEBUG_I2C, "RX: count %u, next_byte %u\r\n", count, device[handle].rx.next_byte);

}

static inline void pca9665_read_data(int handler, uint8_t* dst, int len)
{
	while (len--)
		*dst++ = *(uint8_t *)(device[handler].base + I2CDAT);
}

void pca9665_dump_regs(int handler) 
{
	printf("I2CSTA\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CSTA));
	printf("I2CDAT\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CDAT));
	printf("I2CCON\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CCON));
	printf("I2CCOUNT\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CCOUNT));
	printf("I2CADR\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CADR));
	printf("I2CSCLL\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CSCLL));
	printf("I2CSCLH\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CSCLH));
	printf("I2CTO\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CTO));
	printf("I2CMODE\t\t: 0x%02X\n\r", pca9665_read_reg(handler, I2CMODE));
}

int i2c_master_transaction(int handle, uint8_t addr, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout) 
{

	if (handle >= pca9665_device_count)
		return E_NO_DEVICE;

	if (!device[handle].is_initialised)
		return E_NO_DEVICE;

	if ((txlen > I2C_MTU) || (rxlen > I2C_MTU))
		return E_INVALID_BUF_SIZE;

	i2c_frame_t * frame = (i2c_frame_t *)pvPortMalloc(sizeof(i2c_frame_t));
	if (frame == NULL)
		return E_NO_BUFFER;

	/* Take the I2C lock */
	xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

	/* Temporarily disable the RX callback, because we wish the received message to go into the I2C queue instead */
	void * tmp_callback = device[handle].callback;
	device[handle].callback = NULL;

	frame->dest = addr;
	memcpy(&frame->data[0], txbuf, txlen);
	frame->len = txlen;
	frame->len_rx = rxlen;

	if (i2c_send(handle, frame, 0) != E_NO_ERR) 
	{
		vPortFree(frame);
		device[handle].callback = tmp_callback;
		xSemaphoreGive(i2c_lock);
		return E_TIMEOUT;
	}

	if (rxlen == 0) 
	{
		device[handle].callback = tmp_callback;
		xSemaphoreGive(i2c_lock);
		return E_NO_ERR;
	}

	if (i2c_receive(handle, &frame, timeout) != E_NO_ERR) 
	{
		device[handle].callback = tmp_callback;
		xSemaphoreGive(i2c_lock);
		return E_TIMEOUT;
	}

	memcpy(rxbuf, &frame->data[0], rxlen);

	vPortFree(frame);
	device[handle].callback = tmp_callback;
	xSemaphoreGive(i2c_lock);
	return E_NO_ERR;

}

/**
* Send I2C frame via the selected device
*
* @param handle Handle to the device
* @param frame Pointer to I2C frame
* @param timeout Ticks to wait
* @return Error code as per error.h
*/
int i2c_send(int handle, i2c_frame_t * frame, uint16_t timeout) 
{

	if (handle >= pca9665_device_count)
		return E_NO_DEVICE;

	if (!device[handle].is_initialised)
		return E_NO_DEVICE;

	if (xQueueSendToBack(device[handle].tx.queue, &frame, timeout) == pdFALSE)
		return E_TIMEOUT;

	/* Check state in critical region */
	vPortEnterCritical();
	{
		/* If not currently busy, send the start condition... */
		if (device[handle].is_busy == 0)
			pca9665_write_reg(handle, I2CCON, CON_ENSIO | CON_MODE | CON_AA | CON_STA);
	}
	vPortExitCritical();

	return E_NO_ERR;
}

/**
* receive I2C frame from selected device
* Context: Task only
*
* @param handle Handle to the device
* @param frame Pointer to I2C frame (free this when done!!!)
* @param timeout Number of ticks to wait for a frame
* @return Returns error code: E_NO_ERR if a frame is received, or E_TIMEOUT if timed out, E_NO_DEVICE if handle is not a valid device
*/
int i2c_receive(int handle, i2c_frame_t ** frame, uint16_t timeout) 
{

	if (handle >= pca9665_device_count)
		return E_NO_DEVICE;

	if (!device[handle].is_initialised)
		return E_NO_DEVICE;

	if (device[handle].rx.queue == NULL)
		return E_NO_DEVICE;

	if (xQueueReceive(device[handle].rx.queue, frame, timeout) == pdFALSE)
		return E_TIMEOUT;

	return E_NO_ERR;

}