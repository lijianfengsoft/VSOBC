/***************************************************************************
*  功能：初始化SPI1和3片AD7490
*  版本：
*  迭代：SPI1：	MOSI:PA7
				MISO:A6
				SCLK:PA5
		星务上的AD7490D的片选为：PB1
		电源上第一个AD7490片选为：PA4
		电源上第二个AD7490片选为：PA15
SPI，是一种高速的，全双工，同步的通信总线，并且在芯片的管脚上只占用四根线
SPI接口一般使用4条线通信：
MISO 主设备数据输入，从设备数据输出。
MOSI 主设备数据输出，从设备数据输入。
SCLK时钟信号，由主设备产生。
CS从设备片选信号，由主设备控制。
主机和从机都有一个串行移位寄存器，主机通过向它的SPI串行寄存器写入一个字节来发起一次传输
串行移位寄存器通过MOSI信号线将字节传送给从机，从机也将自己的串行移位寄存器中的内容通过MISO信号线返回给主机。这样，两个移位寄存器中的内容就被交换
外设的写操作和读操作是同步完成的。如果只进行写操作，主机只需忽略接收到的字节；反之，若主机要读取从机的一个字节，就必须发送一个空字节来引发从机的传输
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/13 20:01 
*****************************************************************************/  
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "driver_debug.h"

#include "bsp_ad7490.h"

uint16_t ad7490_data[16];

/**
* Keep a record of the current, or latest used SPI device
*/
spi_chip_t * current_chip = NULL;

/**
* Interrupt Service Routine and DMA semaphore
*/
SemaphoreHandle_t spi_lock;

static spi_chip_t ad7490chip = {
	.Direction = SPI_Direction_2Lines_FullDuplex,		//设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	.Mode = SPI_Mode_Master,							//设置SPI工作模式:设置为主SPI
	.BaudRatePrescaler = SPI_BaudRatePrescaler_256,		//定义波特率预分频的值:波特率预分频值为256
	.DataSize = SPI_DataSize_16b,						//设置SPI的数据大小:SPI发送接收16位帧结构
	.CPOL = SPI_CPOL_High,								//串行同步时钟的空闲状态为高电平
	.CPHA = SPI_CPHA_1Edge,								//串行同步时钟的第1个跳变沿（上升或下降）数据被采样
	.NSS = SPI_NSS_Soft,								//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	.FirstBit = SPI_FirstBit_MSB,						//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	.CRCPolynomial = 7,									//CRC值计算的多项式
	.Cs = 2,											//片选代号，2代表选择星务上的AD7490：PB1	0：代表电源上的AD7490：PA4	0：代表电源上的AD7490：PA15
};

/**************************************************
 *  功能:建立SPI访问锁并初始化相关IO口，最后将所有片选拉高并初始化AD7490
 *  日期:2018/1/13 21:53
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void ad7490_spi_init(void)
{
	spi_lock = NULL;
	spi_lock = xSemaphoreCreateMutex();
	BSP_SPI_IO_init();
	BSP_CS_AHIGH();			//都不片选
	AD7490_Init();
}

/**************************************************
 *  功能:SPI1硬件相关的IO口配置
 *  日期:2018/1/13 22:03
 *  输入:
 *  输出:
 *  注意点:此处开启了SPI1的时钟，故后续配置不需要再次开启SPI1的时钟
 *************************************************/
void BSP_SPI_IO_init(void)
{
	GPIO_InitTypeDef	 GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/*!< SPI pins configuration */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	/*!< Configure BSP Card CS pin in output pushpull mode*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**************************************************
 *  功能:初始化AD7490
 *  日期:2018/1/13 22:10
 *  输入:
 *  输出:
 *  注意点:在从零件读取转换结果的同时，数据在DIN线上传输。 
 * 在DIN线上传输的数据对应于下一次转换的AD7490配置。 每个数据
 *传输需要16个串行时钟。 只有在前12个下降时钟沿（CS下降沿之后）
 *提供的信息才被加载到控制寄存器。 MSB表示数据流中的第一位
 *************************************************/
void AD7490_Init(void)
{
	uint32_t i;
	uint16_t ctr_reg;

	/* write to control register */
	/* configure	to shadow mode,normal power mode normal range ,bin coding format */
	ctr_reg = WRITE | SEQ_CFG | CHANNEL_0 | POWER_NORMAL | RANGE_DOUBLE | DATA_BIN;			//写入控制寄存器的值为1000 0011 1001
	SPI_Write_AD7490(ctr_reg);
	/*此处有问题时可加一个延时试试*/
	/* write shadow register with all channels selected */
	SPI_Write_AD7490(ALL_CHANNEL);
}

/**************************************************
 *  功能:通过spi写一个半字到AD7490，同时收到一个半字
 *  日期:2018/1/14 21:03
 *  输入:
 *  输出:
 *  注意点:write 0x0000 to the ad7490 to read converted value
		the outputs contain the channeel num and adc value in a 16 bits data
		the format is 4 bits channel num MSB and the 12bits LSB is adc value
 *************************************************/
uint16_t SPI_Write_AD7490(uint16_t _ucValue)
{
	uint16_t data = 0;

	data = BSP_SendHalfWord(&ad7490chip, (uint16_t)_ucValue);

	return data;
}


/**************************************************
 *  功能:读取AD7490转换的数值
 *  日期:2018/1/14 21:56
 *  输入:
 *  输出:
 *  注意点:通道设置我自动转换在每次cs拉低的一段时间时
 *************************************************/
uint16_t* AD7490_Read(void)
{
	uint16_t i;
	uint16_t *prxdata;
	uint8_t channel;
	uint16_t rxdata;
	prxdata = ad7490_data;

	for (i = 0; i < 16; i++)
	{
		rxdata = SPI_Write_AD7490(DATA_UPDATE);
		channel = (uint8_t)(rxdata >> 12);
		ad7490_data[channel] = rxdata & 0x0fff;
		printf("ad_data[%2d]	%4x\r\n", channel, ad7490_data[channel]);
	}
	return prxdata;
}

/**************************************************
 *  功能:配置SPI1并发送一个半字数据同时收到一个半字数据
 *  日期:2018/1/13 21:38
 *  输入:spi_chip：spi配置结构体	halfword：待写入的数据
 *  输出:写入的同时从机返回的16位数据
 *  注意点:spi_chip->Cs用来选择不同的片选
 *************************************************/
uint16_t BSP_SendHalfWord(spi_chip_t * spi_chip, uint16_t halfword)
{
	volatile unsigned int j = 100000;
	uint16_t data = 0;

	/*如果SPI有新的配置就要更新配置*/
	if (current_chip != spi_chip) 
	{
		spi_setup_chip(spi_chip);
		current_chip = spi_chip;
	}
	if (spi_lock_dev())		//锁住SPI
		return -1;

	chip_select_n(spi_chip->Cs);	//选择片选
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET && j--);//等待发送区空  
	if (j == 0) 
		driver_debug(DEBUG_SPI, "OVER TIME\n");
	SPI_I2S_SendData(SPI1, halfword);									  //通过外设SPIx发送一个数据
	j = 100000;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET && j--);//等待接收完
	if (j == 0) 
		driver_debug(DEBUG_SPI, "OVER TIME\n");
	data = SPI_I2S_ReceiveData(SPI1);								 //返回通过SPIx最近接收的数据

	chip_select_p(spi_chip->Cs);	//放弃之前的片选
	spi_unlock_dev();				//释放锁

	return data;
}

/**************************************************
 *  功能:更新SPI配置，并使能SPI
 *  日期:2018/1/13 21:42
 *  输入:待写入的配置结构体
 *  输出:
 *  注意点:
 *************************************************/
void spi_setup_chip(spi_chip_t * spi_chip)
{
	SPI_Cmd(SPI1, DISABLE);
	SPI_Init(SPI1, (SPI_InitTypeDef*)spi_chip);
	SPI_Cmd(SPI1, ENABLE);
}

/**************************************************
 *  功能:取走互斥信号量，从而锁定SPI
 *  日期:2018/1/13 21:45
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
int spi_lock_dev(void)
{
	if ((!spi_lock))
		return -1;
	if (xSemaphoreTake(spi_lock, SPI_TIMEOUT) == pdFALSE)
	{
		driver_debug(DEBUG_SPI, "SD_LOCK");
		return -1;
	}
	return 0;
}

/**************************************************
 *  功能:释放信号即释放SPI
 *  日期:2018/1/13 22:00
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void spi_unlock_dev(void) 
{
	if ((!spi_lock))
		return;
	xSemaphoreGive(spi_lock);
}

/**************************************************
 *  功能:片选选取
 *  日期:2018/1/13 21:50
 *  输入:要选择哪一个片选
 *  输出:无
 *  注意点:输入只能为0 1 2  2代表选择星务上的AD7490：PB1	0：代表电源上的AD7490：PA4	1：代表电源上的AD7490：PA15
 *************************************************/
void chip_select_n(uint32_t cs) {
	switch (cs) {
	case 0:
		BSP_CS_AHIGH();
		BSP_CS_LOW(GPIO_Pin_4);
		break;
	case 1:
		BSP_CS_AHIGH();
		BSP_CS_LOW(GPIO_Pin_15);
		break;
	case 2:
		BSP_CS_AHIGH();
		BSP_CS_LOW(GPIO_Pin_1);
		break;
	default:
		driver_debug(DEBUG_SPI, "Chip Select Error<0~2>\n");
		break;
	}
}

/**************************************************
 *  功能:放弃一个片选
 *  日期:2018/1/13 21:59
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void chip_select_p(uint32_t cs) {
	switch (cs) {
	case 0:
		BSP_CS_HIGH(GPIO_Pin_4);
		break;
	case 1:
		BSP_CS_HIGH(GPIO_Pin_15);
		break;
	case 2:
		BSP_CS_HIGH(GPIO_Pin_1);
		break;
	default:
		driver_debug(DEBUG_SPI, "Chip Select Error<0~2>\n");
		break;
	}
}

