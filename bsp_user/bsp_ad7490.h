#pragma once

#define SPI_TIMEOUT	(10 * configTICK_RATE_HZ)	//获取信号的超时时限

/* Select BSP_SPI: Chip Select pin low 注意GPIO_Pin_1=0x0002*/
#define BSP_CS_LOW(PIN)       if(PIN == 2) GPIO_ResetBits(GPIOB, PIN); \
								else GPIO_ResetBits(GPIOA, PIN)
/* Deselect BSP_SPI: Chip Select pin high */
#define BSP_CS_HIGH(PIN)      if(PIN == 2) GPIO_SetBits(GPIOB, PIN); \
								else GPIO_SetBits(GPIOA, PIN)

/* Select BSP_SPI: All chip Select pin low */
#define BSP_CS_ALOW()         {GPIO_ResetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_15); \
								GPIO_ResetBits(GPIOB, GPIO_Pin_1);}
/* Deselect BSP_SPI: All chip Select pin high */
#define BSP_CS_AHIGH()        {GPIO_SetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_15); \
								GPIO_SetBits(GPIOB, GPIO_Pin_1);}


#define WRITE               0x8000  //写入控制寄存器这一位的值决定了后面的11位是否被加载到控制寄存器。 
#define NO_WRITE            0x0000	//如果该位为1，则将以下11位写入控制寄存器; 如果它是0，剩余的11位不加载到控制寄存器，并保持不变。

/*****控制寄存器中的SEQ位与SHADOW位结合使用来控制序器的使用功能并访问影子寄存器
seq:shadow
00:这个配置意味着没有使用顺序功能。 为每个单独的转换选择的模拟输入通道由每个先前写
   入操作中的通道地址位ADD0到ADD3的内容确定。 这种操作模式反映了多通道ADC的正常工作情况，
   无需使用定序器功能，每次写入AD7490时都会选择下一个通道进行转换
01:该配置选择影子寄存器进行编程。 写入控制寄存器之后，下面的写入操作将加载影子寄存器的内容。
   这将对每个连续的有效CS下降沿连续转换的通道序列进行编程。 选择的通道不需要连续。
10：如果以这种方式设置SEQ和SHADOW位，则在完成写入操作后，序列功能不会中断。 这允许控制寄存器
   中的其他位在一个序列中被改变而不终止该周期。
11：此配置与ADD3到ADD0通道地址位结合使用，可对从通道0到选定最终通道的连续序列通道进行连续转换编程
   （由控制寄存器中的通道地址位确定）

**************************************************************************/
#define SEQ_NORMAL          0x0000	
#define SEQ_CFG             0x0080	
#define SEQ_UPDATE          0x4000	

#define SHADOW				0x0008
/*************ADD3 TO ADD0********************/
#define CHANNEL_0           0x0000
#define CHANNEL_1           0x0400
#define CHANNEL_2           0x0800
#define CHANNEL_3           0x0C00
#define CHANNEL_4           0x1000
#define CHANNEL_5           0x1400
#define CHANNEL_6           0x1800
#define CHANNEL_7           0x1C00
#define CHANNEL_8           0x2000
#define CHANNEL_9           0x2400
#define CHANNEL_10          0x2800
#define CHANNEL_11          0x2C00
#define CHANNEL_12          0x3000
#define CHANNEL_13          0x3400
#define CHANNEL_14          0x3800
#define CHANNEL_15          0x3C00
/************PM1 PM0*********************/
#define POWER_NORMAL        0x0300 //Normal operation
#define POWER_FULLDOWN      0x0200 //FULL Shutdown
#define POWER_AUTODOWN      0x0100 //Auto shutdown
#define POWER_AUTOSTAN      0x0000 //Auto standby
/********************************
该位选择要在AD7490上使用的模拟输入范围。 如果设置为0，
模拟输入范围从0 V延伸到2×REFIN。 如果设置为1，则模拟输入
范围从0 V延伸到REFIN（用于下一次转换）。 
对于0 V至2×REFIN，VDD = 4.75 V至5.25 V
***************************************/
#define RANGE_NORMAL        0x0020  
#define RANGE_DOUBLE        0x0000
/********************************
该位选择AD7490用于转换结果的输出编码类型。 
如果该位设置为0，则该部分的输出编码是二进制补码。
如果该位设置为1，则该部分的输出编码是直接二进制
（用于下一次转换）
****************************************/
#define DATA_BIN            0x0010
#define DATA_TWOS           0x0000
/*********************************/
#define ALL_CHANNEL         0xFFFF
#define DATA_UPDATE         0x0000
/*********************************/
#define AD_MODE_SEQ         0x54
#define AD_MODE_SINGLE      0x45

#define ADC_REF        	2500 
#define ADC_FULL_SCALE  4095

typedef struct __attribute__((packed)) 
{
	uint16_t Direction;
	uint16_t Mode;
	uint16_t DataSize;
	uint16_t CPOL;
	uint16_t CPHA;
	uint16_t NSS;
	uint16_t BaudRatePrescaler;
	uint16_t FirstBit;
	uint16_t CRCPolynomial;

	uint32_t Cs;		  		  /*!< Chip select: value from 0 ~ 2*/
} spi_chip_t;

void ad7490_spi_init(void);
void BSP_SPI_IO_init(void);
void AD7490_Init(void);
uint16_t SPI_Write_AD7490(uint16_t _ucValue);
uint16_t* AD7490_Read(void);
uint16_t BSP_SendHalfWord(spi_chip_t * spi_chip, uint16_t halfword);
void spi_setup_chip(spi_chip_t * spi_chip);
int spi_lock_dev(void);
void spi_unlock_dev(void);
void chip_select_n(uint32_t cs);
void chip_select_p(uint32_t cs);


