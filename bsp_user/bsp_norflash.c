/***************************************************************************
*  功能：norflash驱动程序，初始化、读、写、擦除等
*  版本：V1.0
*  迭代：
当使用一个外部异步存储器时，用户必须按照存储器的数据手册给出的时序数据，计算和设置下列参数：
●ADDSET：地址建立时间
●ADDHOLD：地址保持时间
●DATAST：数据建立时间
●ACCMOD：访问模式 这个参数允许 FSMC可以灵活地访问多种异步的静态存储器。共有4种扩展模式允许以不同的时序分别读写存储器。
在扩展模式下，FSMC_BTR用于配置读操作，FSMC_BWR用于配置写操作。(译注：如果读时序与写时序相同，只须使用FSMC_BTR即可。)
如果使用了同步的存储器，用户必须计算和设置下述参数：
●CLKDIV：时钟分频系数
●DATLAT：数据延时
如果存储器支持的话，NOR闪存的读操作可以是同步的，而写操作仍然是异步的
为了得到正确的FSMC时序配置，下列时序应予以考虑：
●最大的读/写访问时间
●不同的FSMC内部延迟
●不同的存储器内部延迟
因此得到：
((ADDSET + 1) + (DATAST + 1)) × HCLK = max (tWC, tRC)
DATAST × HCLK = tWP
DATAST必须满足：
DATAST = (tAVQV + tsu(Data_NE) + tv(A_NE))/HCLK – ADDSET – 4
注意：由于此时硬件促使必须要16位来读写，所以此处所有函数都是用半字来操作，如果后来者要使用字节模式，更改硬件电路后，
	相应函数即可
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/8 13:29 
*****************************************************************************/                                                    

#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_fsmc.h"
#include "bsp_norflash.h"
#include "stm32f4xx_rcc.h"

/*bank2的首地址*/
#define NOR_FLASH_ADDR  			((uint32_t)0x64000000)

/*延时限定*/
#define BlockErase_Timeout    	((uint32_t)0x00A00000)
#define ChipErase_Timeout     	((uint32_t)0x30000000)
#define Program_Timeout       	((uint32_t)0x00001400)

/*PD6是NOR FLASH输出到STM32的忙信号，通过GPIO查询方式判断*/
#define NOR_IS_BUSY()			(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == RESET)

#define  ADDR_SHIFT(Address) ((__IO uint16_t *)(NOR_FLASH_ADDR+(2*(Address))))
#define  NOR_WRITE(Address,Data) (*(__IO uint16_t *)(Address)=(Data))



/**************************************************
 *  功能:NORFLASH的fsmc总线的初始化
 *  日期:2018/1/8 13:52
 *  输入:无
 *  输出:无
 *  注意点:准确计算各个时间
 *************************************************/
void Norflash_Init(void)
{
	GPIO_InitTypeDef				GPIO_InitStruct;
	FSMC_NORSRAMInitTypeDef			FSMC_NORSRAMInitStruct;
	FSMC_NORSRAMTimingInitTypeDef	FSMC_ReadWriteTimingStruct;

	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |
							RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_AHB3PeriphClockCmd(	RCC_AHB3Periph_FSMC, ENABLE);

	/**************************GPIO配置********************************************/
	/*GPIOD*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStruct.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
								GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
								GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_InitStruct);

	/*GPIOE*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStruct.GPIO_Pin =	GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 |
								GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
								GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*GPIOF*/
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
	
	GPIO_InitStruct.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
								GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 |
								GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*GPIOG*/
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_FSMC);

	GPIO_InitStruct.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
								GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;
	GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure PD6 for NOR memory Ready/Busy signal*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**************************FSMC配置********************************************/
	FSMC_ReadWriteTimingStruct.FSMC_AccessMode = FSMC_AccessMode_B;							//访问模式
	FSMC_ReadWriteTimingStruct.FSMC_AddressHoldTime = 0x01;									//地址保持时间
	FSMC_ReadWriteTimingStruct.FSMC_AddressSetupTime = 0x06;								//地址建立时间
	FSMC_ReadWriteTimingStruct.FSMC_BusTurnAroundDuration = 0x00;							//总线反转时间
	FSMC_ReadWriteTimingStruct.FSMC_CLKDivision = 0x00;										//时钟分频
	FSMC_ReadWriteTimingStruct.FSMC_DataLatency = 0x00;										//数据保持时间
	FSMC_ReadWriteTimingStruct.FSMC_DataSetupTime = 0x0C;									//数据建立时间

	FSMC_NORSRAMInitStruct.FSMC_Bank = FSMC_Bank1_NORSRAM2;									//NOR/SRAM的存储块，共4个选项
	FSMC_NORSRAMInitStruct.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;				//是否选择地址和数据复用数据线
	FSMC_NORSRAMInitStruct.FSMC_MemoryType = FSMC_MemoryType_NOR;							//连接到相应存储块的外部存储器类型
	FSMC_NORSRAMInitStruct.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;					//存储器总线宽度
	FSMC_NORSRAMInitStruct.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;				//使能或关闭同步NOR闪存存储器的突发访问模式设置是否使用迸发访问模式(应该就是连续读写模式吧）
	FSMC_NORSRAMInitStruct.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;			//使能或关闭同步WAIT
	FSMC_NORSRAMInitStruct.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;			//设置WAIT信号的有效电平
	FSMC_NORSRAMInitStruct.FSMC_WrapMode = FSMC_WrapMode_Disable;							//设置是否使用环回模式
	FSMC_NORSRAMInitStruct.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;	//设置WAIT信号有效时机
	FSMC_NORSRAMInitStruct.FSMC_WriteOperation = FSMC_WriteOperation_Enable;				//设定是否使能写操作
	FSMC_NORSRAMInitStruct.FSMC_WaitSignal = FSMC_WaitSignal_Disable;						//设定是否使用WAIT信号
	FSMC_NORSRAMInitStruct.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;					//使能或关闭扩展模式，扩展模式用于访问具有不同读写操作时序的存储器，设定是否使用单独的写时序
	FSMC_NORSRAMInitStruct.FSMC_WriteBurst = FSMC_WriteBurst_Disable;						//设定是否使用迸发写模式
	FSMC_NORSRAMInitStruct.FSMC_ReadWriteTimingStruct = &FSMC_ReadWriteTimingStruct;		//设定读写时序
	FSMC_NORSRAMInitStruct.FSMC_WriteTimingStruct = &FSMC_ReadWriteTimingStruct;			//设定读写时序
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStruct);

	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);											//使能FSMC_Bank1_NORSRAM2区
}

/**************************************************
 *  功能:读取NORFLASH的操作ID
 *  日期:2018/1/8 14:57
 *  输入:无
 *  输出:无
 *  注意点:在查看一个norflash是否初始化成功，可先读取其ID
 *************************************************/
void NOR_ReadID(void)
{
	uint16_t ka, kb, kc, kd;

	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);
	NOR_WRITE(ADDR_SHIFT(0x555), 0x90);

	ka = *(__IO uint16_t *)ADDR_SHIFT(0x0000);
	kb = *(__IO uint16_t  *)ADDR_SHIFT(0x0001);
	kc = *(__IO uint16_t  *)ADDR_SHIFT(0x000E);
	kd = *(__IO uint16_t  *)ADDR_SHIFT(0x000F);

	printf("NorFlash ID = %#x, %#x, %#x, %#x\r\n", ka, kb, kc, kd);
}

/**************************************************
 *  功能:擦除某一地址所在的扇区
 *  日期:2018/1/8 15:10
 *  输入:地址
 *  输出:错误码
 *  注意点:
 *************************************************/
NOR_Status NOR_EraseBlock(uint32_t BlockAddr)
{
	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);
	NOR_WRITE(ADDR_SHIFT(0x555), 0x80);
	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);
	//NOR_WRITE(ADDR_SHIFT(BlockAddr), 0x30);			//此做法不是擦除想要擦除的区域
	NOR_WRITE((NOR_FLASH_ADDR + BlockAddr), 0x30);		//正确
	return (NOR_GetStatus(BlockErase_Timeout));
}

/**************************************************
 *  功能:擦除整片norflash
 *  日期:2018/1/8 15:21
 *  输入:无
 *  输出:错误码
 *  注意点:
 *************************************************/
NOR_Status NOR_EraseChip(void)
{
	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);
	NOR_WRITE(ADDR_SHIFT(0x555), 0x80);
	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);
	NOR_WRITE(ADDR_SHIFT(0x555), 0x10);

	return (NOR_GetStatus(ChipErase_Timeout));
}

/**************************************************
 *  功能:复位NORFLASH
 *  日期:2018/1/8 15:35
 *  输入:无
 *  输出:状态码
 *  注意点:
 *************************************************/
NOR_Status Nor_Reset(void)
{

	NOR_WRITE(NOR_FLASH_ADDR, 0xF0);

	return (NOR_SUCCESS);
}

/**************************************************
 *  功能:使回到读状态
 *  日期:2018/1/8 15:37
 *  输入:无
 *  输出:状态码
 *  注意点:
 *************************************************/
NOR_Status NOR_ReturnToReadMode(void)
{
	NOR_WRITE(NOR_FLASH_ADDR, 0xF0);

	return (NOR_SUCCESS);
}


/**************************************************
 *  功能:向某一个地址写入半字
 *  日期:2018/1/8 15:45
 *  输入:WriteAddr：要写入的首地址
 *		 data：待写入的数据
 *  输出:错误码
 *  注意点:
 *************************************************/	
NOR_Status NOR_WriteHalfWord(u32 WriteAddr, u16 Data)
{
	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);
	NOR_WRITE(ADDR_SHIFT(0x555), 0xA0);

	NOR_WRITE((NOR_FLASH_ADDR + WriteAddr), Data);

	return (NOR_GetStatus(Program_Timeout));
}

/**************************************************
 *  功能:读出某一位置的半字
 *  日期:2018/1/8 16:57
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
u16 NOR_ReadHalfWord(u32 ReadAddr)
{
	NOR_WRITE(NOR_FLASH_ADDR+ReadAddr, 0xF0);
	return (*(__IO uint16_t *)((NOR_FLASH_ADDR + ReadAddr)));
}

/**************************************************
 *  功能:写入一个buffer
 *  日期:2018/1/8 15:52
 *  输入:pBuffer：待写入的缓冲区
 *		WriteAddr：待写入的首地址
 *		NumHalfwordToWrite:数据长度/半字(不要用sizeof来计算长度了，这个单位是半字，读类似)
 *  输出:错误码
 *  注意点:直接写，没有擦除的情况下，测试正确
 *************************************************/
NOR_Status NOR_WriteBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite)
{
	NOR_Status status = NOR_ONGOING;

	do
	{
		/* Transfer data to the memory */
		status = NOR_WriteHalfWord(WriteAddr, *pBuffer++);
		WriteAddr = WriteAddr + 2;								//此处地址+2
		NumHalfwordToWrite--;
	} while ((status == NOR_SUCCESS) && (NumHalfwordToWrite != 0));

	return (status);
}

/**************************************************
 *  功能:Writes a half-word buffer to the FSMC NOR memory. This function
 *  日期:2018/1/8 16:17
 *  输入:pBuffer : pointer to buffer.
		 WriteAddr: NOR memory internal address from which the data will be written
		 NumHalfwordToWrite: number of Half words to write.
 *  输出:
 *  注意点:
 *************************************************/
NOR_Status NOR_ProgramBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite)
{
	u32 lastloadedaddress = 0x00;
	u32 currentaddress = 0x00;
	u32 endaddress = 0x00;

	/* Initialize variables */
	currentaddress = WriteAddr;
	endaddress = WriteAddr + NumHalfwordToWrite - 1;
	lastloadedaddress = WriteAddr;

	/* Issue unlock command sequence */
	NOR_WRITE(ADDR_SHIFT(0x555), 0xAA);
	NOR_WRITE(ADDR_SHIFT(0x2AA), 0x55);

	/* Write Write Buffer Load Command */
	NOR_WRITE(ADDR_SHIFT(WriteAddr), 0x0025);
	NOR_WRITE(ADDR_SHIFT(WriteAddr), (NumHalfwordToWrite - 1));

	/* Load Data into NOR Buffer */
	while (currentaddress <= endaddress)
	{
		/* Store last loaded address & data value (for polling) */
		lastloadedaddress = currentaddress;

		NOR_WRITE(ADDR_SHIFT(currentaddress), *pBuffer++);
		currentaddress += 1;
	}

	NOR_WRITE(ADDR_SHIFT(lastloadedaddress), 0x29);

	return(NOR_GetStatus(Program_Timeout));
}


/**************************************************
 *  功能:读一个缓冲区
 *  日期:2018/1/8 16:13
 *  输入:pBuffer：读出的数据缓冲区
		ReadAddr：待读区的首地址
		NumHalfwordToRead：待读数据长度/半字
 *  输出:无
 *  注意点:
 *************************************************/
void NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead)
{

	NOR_WRITE((NOR_FLASH_ADDR + ReadAddr), 0x00F0);

	for (; NumHalfwordToRead != 0x00; NumHalfwordToRead--) /* while there is data to read */
	{
		/* Read a Halfword from the NOR */
		*pBuffer++ = *(vu16 *)((NOR_FLASH_ADDR + ReadAddr));
		ReadAddr = ReadAddr + 2;
	}
}


/**************************************************
 *  功能:获取NORFLASH状态
 *  日期:2018/1/8 16:11
 *  输入:超时时限
 *  输出:错误码
 *  注意点:要看手册才懂为什么要读DQ5和DQ6来和0x0040判断，并要进行两次
 *************************************************/
NOR_Status NOR_GetStatus(u32 Timeout)
{
	u16 val1 = 0x00, val2 = 0x00;
	NOR_Status status = NOR_ONGOING;
	u32 timeout = Timeout;

	/* Poll on NOR memory Ready/Busy signal ------------------------------------*/
	while ((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) != RESET) && (timeout > 0))
	{
		timeout--;
	}

	timeout = Timeout;

	while ((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == RESET) && (timeout > 0))
	{
		timeout--;
	}

	/* Get the NOR memory operation status -------------------------------------*/
	while ((Timeout != 0x00) && (status != NOR_SUCCESS))
	{
		Timeout--;

		/* Read DQ6 and DQ5 */
		val1 = *(vu16 *)(NOR_FLASH_ADDR);
		val2 = *(vu16 *)(NOR_FLASH_ADDR);

		/* If DQ6 did not toggle between the two reads then return NOR_Success */
		if ((val1 & 0x0040) == (val2 & 0x0040))
		{
			return NOR_SUCCESS;
		}

		if ((val1 & 0x0020) != 0x0020)
		{
			status = NOR_ONGOING;
		}

		val1 = *(vu16 *)(NOR_FLASH_ADDR);
		val2 = *(vu16 *)(NOR_FLASH_ADDR);

		if ((val1 & 0x0040) == (val2 & 0x0040))
		{
			return NOR_SUCCESS;
		}
		else if ((val1 & 0x0020) == 0x0020)
		{
			return NOR_ERROR;
		}
	}

	if (Timeout == 0x00)
	{
		status = NOR_TIMEOUT;
	}

	/* Return the operation status */
	return (status);
}


