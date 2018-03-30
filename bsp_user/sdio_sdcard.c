/***************************************************************************
*  功能：实现SD卡的初始化和读写功能的编写
*  版本：V1.0
*  迭代：MBR(Master Boot Record, 主引导记录)位于SD卡的第0扇区(物理),
*		共512个字节. 其中前446个字节为引导代码, 接下来64个字节为分区表,
*		再接下来两个字节为签名, 固定为 0×55, 0xAA.
                                                 南京理工大学微纳卫星中心   
                                                    李剑锋：2017/12/24 20:37 
*****************************************************************************/                                                    

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

#include "sdio_sdcard.h"
#include "string.h"	 	 
#include "usart.h"	  

/*用于sdio初始化的结构体*/
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;

SD_Error CmdError(void);
SD_Error CmdResp7Error(void);
SD_Error CmdResp1Error(u8 cmd);
SD_Error CmdResp3Error(void);
SD_Error CmdResp2Error(void);
SD_Error CmdResp6Error(u8 cmd, u16*prca);
SD_Error SDEnWideBus(u8 enx);
SD_Error IsCardProgramming(u8 *pstatus);
SD_Error FindSCR(u16 rca, u32 *pscr);
u8 convert_from_bytes_to_power_of_two(u16 NumberOfBytes);


static u8 CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;		//SD卡类型（默认为1.x卡）
static u32 CSD_Tab[4], CID_Tab[4], RCA = 0;					//SD卡CSD,CID以及相对地址(RCA)数据
static u8 DeviceMode = SD_DMA_MODE;		   				    //工作模式,注意,工作模式必须通过SD_SetDeviceMode,后才算数.这里只是定义一个默认的模式(SD_DMA_MODE)
static u8 StopCondition = 0; 								//是否发送停止传输标志位,DMA多块读写的时候用到  
volatile SD_Error TransferError = SD_OK;					//数据传输错误标志,DMA读写时使用	    
volatile u8 TransferEnd = 0;								//传输结束标志,DMA读写时使用
SD_CardInfo SDCardInfo;									    //SD卡信息

//SD_ReadDisk/SD_WriteDisk函数专用buf,当这两个函数的数据缓存区地址不是4字节对齐的时候,
//需要用到该数组,确保数据缓存区地址是4字节对齐的.
 u8 SDIO_DATA_BUFFER[512] __attribute__((aligned(4)));

 /**************************************************
  *  功能:复位SDIO寄存器
  *  日期:2017/12/24 21:37
  *  输入:无
  *  输出:无
  *  注意点:POWER--电源控制，低2位有效（11）通电为卡提供时钟
			CLKCR--时钟控制，低15位有效
			ARG--参数寄存器，32位有效
			CMD--命令寄存器，低15位有效，命令状态机一般都是开启
			DTIMER--数据定时器寄存器，32位有效
			DLEN--数据长度寄存器，低25位有效，单位为字节
			DCTRL--数据控制寄存器，低12位有效，在写入此寄存器之前必须先写DTIMER和DLEN寄存器
			ICR--中断清零寄存器，只写，某位置一则将STA状态寄存器中对应为清零（0x7FF）低11位清零（11-21保留）
			MASK--中断屏蔽寄存器，低23位有效，通过置一来确定哪一个状态标志位可以产生中断
  *************************************************/
void SDIO_Register_Deinit()
{
	SDIO->POWER = 0x00000000;
	SDIO->CLKCR = 0x00000000;
	SDIO->ARG = 0x00000000;
	SDIO->CMD = 0x00000000;
	SDIO->DTIMER = 0x00000000;
	SDIO->DLEN = 0x00000000;
	SDIO->DCTRL = 0x00000000;
	SDIO->ICR = 0x00C007FF;
	SDIO->MASK = 0x00000000;
}


/**************************************************
 *  功能:初始化SD卡
 *  日期:2017/12/24 21:21
 *  输入:无
 *  输出:错误代码（0，无错误）
 *  注意点:DMA模式未使用，要使用时则要使能要应该标志位即可
 *************************************************/
SD_Error SD_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	SD_Error errorstatus = SD_OK;
	u8 clkdiv = 0;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);               //SDIO时钟使能

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SDIO, ENABLE);               //SDIO复位与下面DISENABLE对应


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; 	//PC8,9,10,11(SD0-SD3),12(CLK)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//PD2(CMD)

										 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SDIO, DISABLE);                 //SDIO结束复位
																   
	SDIO_Register_Deinit();                                               //SDIO外设寄存器设置

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);

	errorstatus = SD_PowerON();			                                  //SD卡上电
	if (errorstatus == SD_OK)
		errorstatus = SD_InitializeCards();			                      //初始化SD卡														  
	if (errorstatus == SD_OK)
		errorstatus = SD_GetCardInfo(&SDCardInfo);	                      //获取卡信息
	if (errorstatus == SD_OK)
		errorstatus = SD_SelectDeselect((u32)(SDCardInfo.RCA << 16));     //选中SD卡   
	if (errorstatus == SD_OK)
		errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_1b);	      //4位宽度,如果是MMC卡,则不能用4位模式 
	if ((errorstatus == SD_OK) || (SDIO_MULTIMEDIA_CARD == CardType))
	{
		if (SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1 || SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0)
		{
			clkdiv = SDIO_TRANSFER_CLK_DIV + 2;	                          //V1.1/V2.0卡，设置最高48/4=12Mhz
		}
		else clkdiv = SDIO_TRANSFER_CLK_DIV+2;	                          //SDHC等其他卡，设置最高48/2=24Mhz
		SDIO_Clock_Set(clkdiv);	                                          //设置时钟频率,SDIO时钟计算公式:SDIO_CK时钟=SDIOCLK/[clkdiv+2];其中,SDIOCLK固定为48Mhz 
        //errorstatus=SD_SetDeviceMode(SD_DMA_MODE);	                  //设置为DMA模式
		errorstatus = SD_SetDeviceMode(SD_POLLING_MODE);                  //设置为查询模式
	}
	return errorstatus;
}

/**************************************************
 *  功能:SDIO时钟初始化
 *  日期:2017/12/25 10:39
 *  输入:clkdiv:时钟分频系数
 *  输出:无
 *  注意点:CK时钟=SDIOCLK/[clkdiv+2];(SDIOCLK时钟固定为48Mhz)
 *************************************************/
void SDIO_Clock_Set(u8 clkdiv)
{
	u32 tmpreg = SDIO->CLKCR;
	tmpreg &= 0XFFFFFF00;
	tmpreg |= clkdiv;
	SDIO->CLKCR = tmpreg;
}


/**************************************************
 *  功能:SD卡初始化第一步为上电，
 *		 SDIO初始化，配置三种时钟
 *       查询所有SDIO接口上的卡设备,并查询其电压和配置时钟
 *  输入:无
 *  输出:错误代码;(0,无错误)
 *  注意点:初始化时卡时钟不能大于400KHz，三个时钟，APB2总线上时钟PLCK2，SDIO适配器时钟SDIOCLK,卡时钟SDIO_CK
 *         默认为1位数据宽SD0		
 *         命令和响应都是在CMD线上传输，数据在SD线上传输
 *************************************************/
SD_Error SD_PowerON(void)
{
	u8 i = 0;
	SD_Error errorstatus = SD_OK;
	u32 response = 0, count = 0, validvoltage = 0;
	u32 SDType = SD_STD_CAPACITY;                                                      //开始假设为标准容量卡

	SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;	                           //PLL48CK = 48MHz, SDIOCLK = 48MHz, SDIO_CK = SDIOCLK/(118 + 2) = 400 KHz
	SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
	SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;                    //不使用bypass模式，直接用PLL48CK进行分频得到SDIO_CK
	SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;	           // 空闲时不关闭时钟电源
	SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;	 				               //1位数据线
	SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;    //硬件流
	SDIO_Init(&SDIO_InitStructure);

	SDIO_SetPowerState(SDIO_PowerState_ON);	                                           //上电从而开启卡时钟   
	SDIO->CLKCR |= 1 << 8;			                                                   //SDIOCK使能  

	for (i = 0; i < 74; i++)															//尝试多次发送CMD0，进入卡复位模式
	{

		SDIO_CmdInitStructure.SDIO_Argument = 0x0;                                     //发送CMD0进入IDLE STAGE模式命令.
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE;                    
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;                        //无响应
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;								   //所有命令发送都设置不等待
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;                            //CPSM命令状态机都设置为开启。 
		SDIO_SendCommand(&SDIO_CmdInitStructure);	  		                           //写命令进命令寄存器

		errorstatus = CmdError();													   //CMD0执行状态	

		if (errorstatus == SD_OK)
			break;
	}
	if (errorstatus)
		return errorstatus;

	SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;                        	 //发送CMD8,短响应,检查SD卡接口特性参数1AA（1电压为2.6-3.6）
	SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;	                     //cmd8 区分卡型号和核对电压
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;	                     //r7
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;			                        
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp7Error();						                             //等待R7响应

	if (errorstatus == SD_OK) 								                         //R7响应正常
	{
		CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0;		                             //SD 2.0卡
		SDType = SD_HIGH_CAPACITY;			   			                             //高容量卡？？？？？？（ACMD41参数使用）
	}

	/*此处发送CMD55有点多余（后期测试看是否可删除）*/
	SDIO_CmdInitStructure.SDIO_Argument = 0x00;                                      //发送CMD55,短响应，在发送应用命令之前都要发送cmd55	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;						 //r1响应
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);		 

	errorstatus = CmdResp1Error(SD_CMD_APP_CMD); 		 	                         //等待R1响应   

	if (errorstatus == SD_OK)
	{
		//SD卡,发送ACMD41 SD_APP_OP_COND,参数为:0x80100000 
		while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
		{
			SDIO_CmdInitStructure.SDIO_Argument = 0x00;                              //发送CMD55,短响应
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;	                 //CMD55
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);			 

			errorstatus = CmdResp1Error(SD_CMD_APP_CMD); 	 	                      //等待R1响应  

			if (errorstatus != SD_OK)
				return errorstatus;   	

	//acmd41，命令参数由支持的电压范围及HCS位组成，HCS位置一来区分是否支持高容量卡
			SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;	  //发送ACMD41,短响应	
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;                //r3短响应
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp3Error(); 					                         //等待R3响应，响应成功包含SD卡OCR寄存器内容，OCRbit30：CCS位表示是SDHC还是SDSC，对1.x忽略此位，bit31：SD是否上电成功   

			if (errorstatus != SD_OK) 
				return errorstatus;   	
			response = SDIO->RESP1;;			   				                    //得到响应
			validvoltage = (((response >> 31) == 1) ? 1 : 0);			            //判断SD卡上电是否完成bit31
			count++;
		}
		if (count >= SD_MAX_VOLT_TRIAL)
		{
			errorstatus = SD_INVALID_VOLTRANGE;
			return errorstatus;
		}
		if (response &= SD_HIGH_CAPACITY)											 //bit30
		{
			CardType = SDIO_HIGH_CAPACITY_SD_CARD;
		}
	}
	else
	{
		//MMC卡,发送CMD1 SDIO_SEND_OP_COND,参数为:0x80FF8000 
		while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
		{
			SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_MMC;           //发送CMD1,短响应	   
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_OP_COND;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;             //r3
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp3Error(); 					                      //等待R3响应   

			if (errorstatus != SD_OK)
				return errorstatus;   	           
			response = SDIO->RESP1;;			   				                  //得到响应
			validvoltage = (((response >> 31) == 1) ? 1 : 0);
			count++;
		}
		if (count >= SD_MAX_VOLT_TRIAL)
		{
			errorstatus = SD_INVALID_VOLTRANGE;
			return errorstatus;
		}
		CardType = SDIO_MULTIMEDIA_CARD;
	}
	return(errorstatus);
}


/**************************************************
 *  功能:关闭SD卡，掉电，时钟停止
 *  日期:2017/12/25 10:47
 *  输入:无
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error SD_PowerOFF(void)
{

	SDIO_SetPowerState(SDIO_PowerState_OFF);//SDIO电源关闭,时钟停止	

	return SD_OK;
}

/**************************************************
 *  功能:初始化所有的卡，并让卡进入就绪状态
 *  日期:2017/12/24 23:53
 *  输入:无
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error SD_InitializeCards(void)
{
	SD_Error errorstatus = SD_OK;
	u16 rca = 0x01;														   //SD卡相对地址

	if (SDIO_GetPowerState() == SDIO_PowerState_OFF)                       //检查电源状态,确保为上电状态
	{
		errorstatus = SD_REQUEST_NOT_APPLICABLE;
		return(errorstatus);
	}

	if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)		
	{
		SDIO_CmdInitStructure.SDIO_Argument = 0x0;                           //发送CMD2,取得SD卡CID寄存器信息,长响应
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp2Error(); 					                     //等待R2响应 

		if (errorstatus != SD_OK)
			return errorstatus;   		 

		CID_Tab[0] = SDIO->RESP1;
		CID_Tab[1] = SDIO->RESP2;
		CID_Tab[2] = SDIO->RESP3;
		CID_Tab[3] = SDIO->RESP4;
	}
	if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))//判断卡类型
	{
		SDIO_CmdInitStructure.SDIO_Argument = 0x00;                          //发送CMD3,短响应 获取RCA寄存器的值，卡的相对地址
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;	
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;           //r6
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	                        //发送CMD3,短响应 

		errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);             //等待R6响应 获取RCA到rca

		if (errorstatus != SD_OK)
			return errorstatus;   	           	    
	}
	if (SDIO_MULTIMEDIA_CARD == CardType)
	{
		/*MMC卡位主机设置rca而不是sd卡返回rca*/
		SDIO_CmdInitStructure.SDIO_Argument = (u32)(rca << 16);             //发送CMD3,短响应 参数为要设置的RCA
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;	     
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; 
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	

		errorstatus = CmdResp2Error(); 					                    //等待R2响应 ？？？？（CMD3有两种响应）  

		if (errorstatus != SD_OK)
			return errorstatus;   	 
	}
	if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)	
	{
		RCA = rca;                                                           //选择卡

		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);         //发送CMD9+卡RCA,取得SD卡CSD寄存器,长响应 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_CSD;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;			 //r2响应？？？
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp2Error(); 					                     //等待R2响应   
		if (errorstatus != SD_OK)
			return errorstatus;   		    

		CSD_Tab[0] = SDIO->RESP1;
		CSD_Tab[1] = SDIO->RESP2;
		CSD_Tab[2] = SDIO->RESP3;
		CSD_Tab[3] = SDIO->RESP4;
	}
	return SD_OK;                                                            //卡初始化成功
}

/**************************************************
 *  功能:获取卡信息
 *  日期:2017/12/25 0:21
 *  输入:SD卡信息结构体
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
	SD_Error errorstatus = SD_OK;
	u8 tmp = 0;
	cardinfo->CardType = (u8)CardType; 				             //卡类型
	cardinfo->RCA = (u16)RCA;							         //卡RCA值
	tmp = (u8)((CSD_Tab[0] & 0xFF000000) >> 24);
	cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;		         //CSD结构
	cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;	     //2.0协议还没定义这部分(为保留),应该是后续协议定义的
	cardinfo->SD_csd.Reserved1 = tmp & 0x03;			         //2个保留位  
	tmp = (u8)((CSD_Tab[0] & 0x00FF0000) >> 16);			     //第1个字节
	cardinfo->SD_csd.TAAC = tmp;				   		         //数据读时间1
	tmp = (u8)((CSD_Tab[0] & 0x0000FF00) >> 8);	  		         //第2个字节
	cardinfo->SD_csd.NSAC = tmp;		  				         //数据读时间2
	tmp = (u8)(CSD_Tab[0] & 0x000000FF);				         //第3个字节
	cardinfo->SD_csd.MaxBusClkFrec = tmp;		  		         //传输速度	   
	tmp = (u8)((CSD_Tab[1] & 0xFF000000) >> 24);			     //第4个字节
	cardinfo->SD_csd.CardComdClasses = tmp << 4;    	         //卡指令类高四位
	tmp = (u8)((CSD_Tab[1] & 0x00FF0000) >> 16);	 		     //第5个字节
	cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;       //卡指令类低四位
	cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;	    	         //最大读取数据长度
	tmp = (u8)((CSD_Tab[1] & 0x0000FF00) >> 8);			         //第6个字节
	cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;	         //允许分块读
	cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;	     //写块错位
	cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;	     //读块错位
	cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
	cardinfo->SD_csd.Reserved2 = 0; 					          //保留
	if ((CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0) || (SDIO_MULTIMEDIA_CARD == CardType))//标准1.1/2.0卡/MMC卡
	{
		cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;	      //C_SIZE(12位)
		tmp = (u8)(CSD_Tab[1] & 0x000000FF); 			          //第7个字节	
		cardinfo->SD_csd.DeviceSize |= (tmp) << 2;
		tmp = (u8)((CSD_Tab[2] & 0xFF000000) >> 24);		      //第8个字节	
		cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;
		cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
		cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);
		tmp = (u8)((CSD_Tab[2] & 0x00FF0000) >> 16);		      //第9个字节	
		cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
		cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
		cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;       //C_SIZE_MULT
		tmp = (u8)((CSD_Tab[2] & 0x0000FF00) >> 8);	  	          //第10个字节	
		cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;
		cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1);//计算卡容量
		cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
		cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);//块大小
		cardinfo->CardCapacity *= cardinfo->CardBlockSize;
	}
	else if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)	         //高容量卡
	{
		tmp = (u8)(CSD_Tab[1] & 0x000000FF); 		            //第7个字节	
		cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;       //C_SIZE
		tmp = (u8)((CSD_Tab[2] & 0xFF000000) >> 24); 	        //第8个字节	
		cardinfo->SD_csd.DeviceSize |= (tmp << 8);
		tmp = (u8)((CSD_Tab[2] & 0x00FF0000) >> 16);	        //第9个字节	
		cardinfo->SD_csd.DeviceSize |= (tmp);
		tmp = (u8)((CSD_Tab[2] & 0x0000FF00) >> 8); 	        //第10个字节	
		cardinfo->CardCapacity = (long long)(cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;//计算卡容量
		cardinfo->CardBlockSize = 512; 			               //块大小固定为512字节****
	}
	cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
	cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;
	tmp = (u8)(CSD_Tab[2] & 0x000000FF);			            //第11个字节	
	cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
	cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);
	tmp = (u8)((CSD_Tab[3] & 0xFF000000) >> 24);		        //第12个字节	
	cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
	cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
	cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
	cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;
	tmp = (u8)((CSD_Tab[3] & 0x00FF0000) >> 16);		       //第13个字节
	cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
	cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
	cardinfo->SD_csd.Reserved3 = 0;
	cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);
	tmp = (u8)((CSD_Tab[3] & 0x0000FF00) >> 8);		           //第14个字节
	cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
	cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
	cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
	cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
	cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
	cardinfo->SD_csd.ECC = (tmp & 0x03);
	tmp = (u8)(CSD_Tab[3] & 0x000000FF);			          //第15个字节
	cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
	cardinfo->SD_csd.Reserved4 = 1;
	tmp = (u8)((CID_Tab[0] & 0xFF000000) >> 24);		     //第0个字节
	cardinfo->SD_cid.ManufacturerID = tmp;
	tmp = (u8)((CID_Tab[0] & 0x00FF0000) >> 16);		     //第1个字节
	cardinfo->SD_cid.OEM_AppliID = tmp << 8;
	tmp = (u8)((CID_Tab[0] & 0x000000FF00) >> 8);		     //第2个字节
	cardinfo->SD_cid.OEM_AppliID |= tmp;
	tmp = (u8)(CID_Tab[0] & 0x000000FF);			         //第3个字节	
	cardinfo->SD_cid.ProdName1 = tmp << 24;
	tmp = (u8)((CID_Tab[1] & 0xFF000000) >> 24); 		     //第4个字节
	cardinfo->SD_cid.ProdName1 |= tmp << 16;
	tmp = (u8)((CID_Tab[1] & 0x00FF0000) >> 16);	      	//第5个字节
	cardinfo->SD_cid.ProdName1 |= tmp << 8;
	tmp = (u8)((CID_Tab[1] & 0x0000FF00) >> 8);			    //第6个字节
	cardinfo->SD_cid.ProdName1 |= tmp;
	tmp = (u8)(CID_Tab[1] & 0x000000FF);	  				//第7个字节
	cardinfo->SD_cid.ProdName2 = tmp;
	tmp = (u8)((CID_Tab[2] & 0xFF000000) >> 24); 			//第8个字节
	cardinfo->SD_cid.ProdRev = tmp;
	tmp = (u8)((CID_Tab[2] & 0x00FF0000) >> 16);			//第9个字节
	cardinfo->SD_cid.ProdSN = tmp << 24;
	tmp = (u8)((CID_Tab[2] & 0x0000FF00) >> 8); 			//第10个字节
	cardinfo->SD_cid.ProdSN |= tmp << 16;
	tmp = (u8)(CID_Tab[2] & 0x000000FF);   					//第11个字节
	cardinfo->SD_cid.ProdSN |= tmp << 8;
	tmp = (u8)((CID_Tab[3] & 0xFF000000) >> 24); 			//第12个字节
	cardinfo->SD_cid.ProdSN |= tmp;
	tmp = (u8)((CID_Tab[3] & 0x00FF0000) >> 16);	 		//第13个字节
	cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
	cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;
	tmp = (u8)((CID_Tab[3] & 0x0000FF00) >> 8);				//第14个字节
	cardinfo->SD_cid.ManufactDate |= tmp;
	tmp = (u8)(CID_Tab[3] & 0x000000FF);					//第15个字节
	cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
	cardinfo->SD_cid.Reserved2 = 1;
	return errorstatus;
}

/**************************************************
 *  功能:设置SDIO总线宽度
 *  日期:2017/12/25 9:35
 *  输入:位宽模式.0,1位数据宽度;1,4位数据宽度;2,8位数据宽度
 *  @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
 *  @arg SDIO_BusWide_4b: 4-bit data transfer
 *  @arg SDIO_BusWide_1b: 1-bit data transfer (默认)
 *  输出:错误码
 *  注意点:(MMC卡不支持4bit模式)
 *************************************************/
SD_Error SD_EnableWideBusOperation(u32 WideMode)
{
	SD_Error errorstatus = SD_OK;
	if (SDIO_MULTIMEDIA_CARD == CardType)
	{
		errorstatus = SD_UNSUPPORTED_FEATURE;
		return(errorstatus);
	}

	else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
	{
		if (SDIO_BusWide_8b == WideMode)                         //2.0 sd不支持8bits
		{
			errorstatus = SD_UNSUPPORTED_FEATURE;
			return(errorstatus);
		}
		else
		{
			errorstatus = SDEnWideBus(WideMode);				//设置总线宽度
			if (SD_OK == errorstatus)
			{
				SDIO->CLKCR &= ~(3 << 11);		                //清除之前的位宽设置    
				SDIO->CLKCR |= WideMode;                        //1位/4位总线宽度 
				SDIO->CLKCR |= 0 << 14;			                //不开启硬件流控制
			}
		}
	}
	return errorstatus;
}

/**************************************************
 *  功能:设置SD卡工作模式
 *  日期:2017/12/25 10:41
 *  输入:DMA或查询模式
 *  输出:错误码
 *  注意点:本次为查询模式，可通过修改次函数来开启DMA模式
 *************************************************/
SD_Error SD_SetDeviceMode(u32 Mode)
{
	SD_Error errorstatus = SD_OK;
	if ((Mode == SD_DMA_MODE) || (Mode == SD_POLLING_MODE))
		DeviceMode = Mode;
	else errorstatus = SD_INVALID_PARAMETER;
	return errorstatus;
}

/**************************************************
 *  功能:选卡，发送CMD7,选择相对地址(rca)为addr的卡,取消其他卡.如果为0,则都不选择.
 *  日期:2017/12/25 0:22
 *  输入:RCA地址
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error SD_SelectDeselect(u32 addr)
{

	SDIO_CmdInitStructure.SDIO_Argument = addr;                         //发送CMD7,选择卡,短响应参数为RCA卡相对地址	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	return CmdResp1Error(SD_CMD_SEL_DESEL_CARD);						//等待R1响应
}

/**************************************************
 *  功能:读取SD卡一个块
 *  日期:2017/12/25 10:48
 *  输入:buf:读数据缓存区(必须4字节对齐!!)；addr:读取地址；blksize:块大小
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error SD_ReadBlock(u8 *buf, long long addr, u16 blksize)
{
	SD_Error errorstatus = SD_OK;
	u8 power;
	u32 count = 0, *tempbuff = (u32*)buf;                             //转换为u32指针 
	u32 timeout = SDIO_DATATIMEOUT;
	if (NULL == buf)
		return SD_INVALID_PARAMETER;
	SDIO->DCTRL = 0x0;	                                              //数据控制寄存器清零(关DMA) 

	if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)                       //大容量卡
	{
		blksize = 512;
		addr >>= 9;
	}
	SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);


	if (SDIO->RESP1&SD_CARD_LOCKED)									  //卡锁了
		return SD_LOCK_UNLOCK_FAILED;      
	if ((blksize > 0) && (blksize <= 2048) && ((blksize&(blksize - 1)) == 0))
	{
		power = convert_from_bytes_to_power_of_two(blksize);

		//发送CMD16+设置数据长度为blksize,短响应
		SDIO_CmdInitStructure.SDIO_Argument = blksize;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);


		errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);	          //等待R1响应 

		if (errorstatus != SD_OK)
			return errorstatus;             
	}
	else return SD_INVALID_PARAMETER;

	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4;          //清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength = blksize;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	//发送CMD17+从addr地址出读取数据,短响应 
	SDIO_CmdInitStructure.SDIO_Argument = addr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);                            //等待R1响应   
	if (errorstatus != SD_OK)
		return errorstatus;   			 
	if (DeviceMode == SD_POLLING_MODE)						                          //查询模式,轮询数据	 
	{
		taskENTER_CRITICAL();                                                               //关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
		while (!(SDIO->STA&((1 << 5) | (1 << 1) | (1 << 3) | (1 << 10) | (1 << 9))))  //无上溢/CRC/超时/完成(标志)/起始位错误
		{
			if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)					  //接收区半满,表示至少存了8个字
			{
				for (count = 0; count < 8; count++)			                          //循环读取数据
				{
					*(tempbuff + count) = SDIO->FIFO;								  //******************************
				}
				tempbuff += 8;
				timeout = 0X7FFFFF; 	                                              //读数据溢出时间
			}
			else 	                                                                  //处理超时
			{
				if (timeout == 0)
					return SD_DATA_TIMEOUT;
				timeout--;
			}
		}
		if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		                  //数据超时错误
		{
			SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 	                                  //清错误标志
			return SD_DATA_TIMEOUT;
		}
		else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	                  //数据块CRC错误
		{
			SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		                              //清错误标志
			return SD_DATA_CRC_FAIL;
		}
		else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) 	                  //接收fifo上溢错误
		{
			SDIO_ClearFlag(SDIO_FLAG_RXOVERR);		                                  //清错误标志
			return SD_RX_OVERRUN;
		}
		else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	                  //接收起始位错误
		{
			SDIO_ClearFlag(SDIO_FLAG_STBITERR);                                       //清错误标志
			return SD_START_BIT_ERR;
		}
		while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)	                       //FIFO里面,还存在可用数据
		{
			*tempbuff = SDIO->FIFO;	                                                   //循环读取数据
			tempbuff++;
		}
		taskEXIT_CRITICAL();                                                                 //开启总中断
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                             //清除所有标记

	}
	else if (DeviceMode == SD_DMA_MODE)
	{
		TransferError = SD_OK;
		StopCondition = 0;			                                                  //单块读,不需要发送停止传输指令
		TransferEnd = 0;				                                              //传输结束标置位，在中断服务置1
		SDIO->MASK |= (1 << 1) | (1 << 3) | (1 << 8) | (1 << 5) | (1 << 9);	          //配置需要的中断 
		SDIO->DCTRL |= 1 << 3;		 	                                              //SDIO DMA使能 
		SD_DMA_Config((u32*)buf, blksize, DMA_DIR_PeripheralToMemory);
		while (((DMA2->LISR&(1 << 27)) == RESET) && (TransferEnd == 0) && (TransferError == SD_OK) && timeout)
			timeout--;                                                                //等待传输完成 
		if (timeout == 0)
			return SD_DATA_TIMEOUT;                                                   //超时
		if (TransferError != SD_OK)
			errorstatus = TransferError;
	}
	return errorstatus;
}

/**************************************************
 *  功能:SD卡读取多个块
 *  日期:2017/12/25 11:12
 *  输入:buf:读数据缓存区 addr:读取地址 nblks:要读取的块数
 *  输出: 错误状态
 *  注意点:
 *************************************************/
 u32 *tempbuff __attribute__((aligned(4)));                                //必须4字节对齐
SD_Error SD_ReadMultiBlocks(u8 *buf, long long addr, u16 blksize, u32 nblks)
{
	SD_Error errorstatus = SD_OK;
	u8 power;
	u32 count = 0;
	u32 timeout = SDIO_DATATIMEOUT;
	tempbuff = (u32*)buf;                                                 //转换为u32指针

	SDIO->DCTRL = 0x0;		                                              //数据控制寄存器清零(关DMA)   
	if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)                           //大容量卡
	{
		blksize = 512;
		addr >>= 9;
	}

	SDIO_DataInitStructure.SDIO_DataBlockSize = 0; ;                      //清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	if (SDIO->RESP1&SD_CARD_LOCKED)
		return SD_LOCK_UNLOCK_FAILED;                                     //卡锁了
	if ((blksize > 0) && (blksize <= 2048) && ((blksize&(blksize - 1)) == 0))
	{
		power = convert_from_bytes_to_power_of_two(blksize);

		//发送CMD16+设置数据长度为blksize,短响应
		SDIO_CmdInitStructure.SDIO_Argument = blksize; 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);	            //等待R1响应  

		if (errorstatus != SD_OK)
			return errorstatus;   		 

	}
	else 
		return SD_INVALID_PARAMETER;

	if (nblks > 1)										                 //多块读  
	{
		if (nblks*blksize > SD_MAX_DATA_LENGTH)
			return SD_INVALID_PARAMETER;                                //判断是否超过最大接收长度 

		SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4; ;       //nblks*blksize,512块大小,卡到控制器
		SDIO_DataInitStructure.SDIO_DataLength = nblks*blksize;
		SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
		SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
		SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
		SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
		SDIO_DataConfig(&SDIO_DataInitStructure);

		//发送CMD18+从addr地址出读取数据,短响应 
		SDIO_CmdInitStructure.SDIO_Argument = addr;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_READ_MULT_BLOCK);           //等待R1响应 

		if (errorstatus != SD_OK)
			return errorstatus;   	 	 

		if (DeviceMode == SD_POLLING_MODE)
		{
			taskENTER_CRITICAL();                                            //关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
			while (!(SDIO->STA&((1 << 5) | (1 << 1) | (1 << 3) | (1 << 8) | (1 << 9))))//无上溢/CRC/超时/完成(标志)/起始位错误
			{
				if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)   //接收区半满,表示至少存了8个字
				{
					for (count = 0; count < 8; count++)			       //循环读取数据
					{
						*(tempbuff + count) = SDIO->FIFO;
					}
					tempbuff += 8;
					timeout = 0X7FFFFF; 	                          //读数据溢出时间
				}
				else 	
				{
					if (timeout == 0)
						return SD_DATA_TIMEOUT;
					timeout--;
				}
			}
			if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)	//数据超时错误
			{
				SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 	             //清错误标志
				return SD_DATA_TIMEOUT;
			}
			else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)//数据块CRC错误
			{
				SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		        //清错误标志
				return SD_DATA_CRC_FAIL;
			}
			else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) //接收fifo上溢错误
			{
				SDIO_ClearFlag(SDIO_FLAG_RXOVERR);		             //清错误标志
				return SD_RX_OVERRUN;
			}
			else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) //接收起始位错误
			{
				SDIO_ClearFlag(SDIO_FLAG_STBITERR);                  //清错误标志
				return SD_START_BIT_ERR;
			}

			while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)	//FIFO里面,还存在可用数据
			{
				*tempbuff = SDIO->FIFO;	                           //循环读取数据
				tempbuff++;
			}
			if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)	   //接收结束
			{
				if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
				{
					//发送CMD12+结束传输
					SDIO_CmdInitStructure.SDIO_Argument = 0;     
					SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
					SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
					SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
					SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
					SDIO_SendCommand(&SDIO_CmdInitStructure);

					errorstatus = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);//等待R1响应   

					if (errorstatus != SD_OK)
						return errorstatus;
				}
			}
			taskEXIT_CRITICAL();                                     //开启总中断
			SDIO_ClearFlag(SDIO_STATIC_FLAGS);                 //清除所有标记
		}
		else if (DeviceMode == SD_DMA_MODE)
		{
			TransferError = SD_OK;
			StopCondition = 1;			                    //多块读,需要发送停止传输指令 
			TransferEnd = 0;				                //传输结束标置位，在中断服务置1
			SDIO->MASK |= (1 << 1) | (1 << 3) | (1 << 8) | (1 << 5) | (1 << 9);	//配置需要的中断 
			SDIO->DCTRL |= 1 << 3;		 					//SDIO DMA使能 
			SD_DMA_Config((u32*)buf, nblks*blksize, DMA_DIR_PeripheralToMemory);
			while (((DMA2->LISR&(1 << 27)) == RESET) && timeout)timeout--;//等待传输完成 
			if (timeout == 0)
				return SD_DATA_TIMEOUT;                      //超时
			while ((TransferEnd == 0) && (TransferError == SD_OK));
			if (TransferError != SD_OK)errorstatus = TransferError;
		}
	}
	return errorstatus;
}

/**************************************************
 *  功能:SD卡写入一个块
 *  日期:2017/12/25 15:24
 *  输入:buf:数据缓存区 addr:写地址 blksize:块大小	 
 *  输出:错误状态
 *  注意点:
 *************************************************/
SD_Error SD_WriteBlock(u8 *buf, long long addr, u16 blksize)
{
	SD_Error errorstatus = SD_OK;

	u8  power = 0, cardstate = 0;

	u32 timeout = 0, bytestransferred = 0;

	u32 cardstatus = 0, count = 0, restwords = 0;

	u32	tlen = blksize;						                                //总长度(字节)

	u32*tempbuff = (u32*)buf;

	if (buf == NULL)
		return SD_INVALID_PARAMETER;                                        //参数错误  

	SDIO->DCTRL = 0x0;							                            //数据控制寄存器清零(关DMA)

	SDIO_DataInitStructure.SDIO_DataBlockSize = 0;                          //清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);


	if (SDIO->RESP1&SD_CARD_LOCKED)
		return SD_LOCK_UNLOCK_FAILED;                                        //卡锁了
	if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)	                             //大容量卡
	{
		blksize = 512;
		addr >>= 9;
	}
	if ((blksize > 0) && (blksize <= 2048) && ((blksize&(blksize - 1)) == 0))
	{
		power = convert_from_bytes_to_power_of_two(blksize);

		//发送CMD16+设置数据长度为blksize,短响应 
		SDIO_CmdInitStructure.SDIO_Argument = blksize;	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);	                 //等待R1响应  

		if (errorstatus != SD_OK)
			return errorstatus;   		 
	}
	else
		return SD_INVALID_PARAMETER;

	//发送CMD13,查询卡的状态,短响应 	
	SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA << 16;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);		                //等待R1响应  

	if (errorstatus != SD_OK)
		return errorstatus;
	cardstatus = SDIO->RESP1;
	timeout = SD_DATATIMEOUT;
	while (((cardstatus & 0x00000100) == 0) && (timeout > 0)) 	           //检查READY_FOR_DATA位是否置位
	{
		timeout--;

		SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA << 16;              //发送CMD13,查询卡的状态,短响应
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);	              //等待R1响应   

		if (errorstatus != SD_OK)
			return errorstatus;

		cardstatus = SDIO->RESP1;
	}
	if (timeout == 0)
		return SD_ERROR;

	//发送CMD24,写单块指令,短响应
	SDIO_CmdInitStructure.SDIO_Argument = addr; 	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);              //等待R1响应  

	if (errorstatus != SD_OK)
		return errorstatus;

	StopCondition = 0;									                //单块写,不需要发送停止传输指令 

	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4; ;	        //blksize, 控制器到卡	
	SDIO_DataInitStructure.SDIO_DataLength = blksize;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);


	timeout = SDIO_DATATIMEOUT;

	if (DeviceMode == SD_POLLING_MODE)
	{
		taskENTER_CRITICAL();                                                             //关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
		while (!(SDIO->STA&((1 << 10) | (1 << 4) | (1 << 1) | (1 << 3) | (1 << 9))))//数据块发送成功/下溢/CRC/超时/起始位错误
		{
			if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)					 //发送区半空,表示至少存了8个字
			{
				if ((tlen - bytestransferred) < SD_HALFFIFOBYTES)                    //不够32字节了
				{
					restwords = ((tlen - bytestransferred) % 4 == 0) ? ((tlen - bytestransferred) / 4) : ((tlen - bytestransferred) / 4 + 1);

					for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
					{
						SDIO->FIFO = *tempbuff;
					}
				}
				else
				{
					for (count = 0; count < 8; count++)
					{
						SDIO->FIFO = *(tempbuff + count);
					}
					tempbuff += 8;
					bytestransferred += 32;
				}
				timeout = 0X3FFFFFFF;	                                           //写数据溢出时间
			}
			else
			{
				if (timeout == 0)
					return SD_DATA_TIMEOUT;
				timeout--;
			}
		}
		if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		               //数据超时错误
		{
			SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 	                               //清错误标志
			return SD_DATA_TIMEOUT;
		}
		else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	               //数据块CRC错误
		{
			SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		                           //清错误标志
			return SD_DATA_CRC_FAIL;
		}
		else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	               //接收fifo下溢错误
		{
			SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);		                               //清错误标志
			return SD_TX_UNDERRUN;
		}
		else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	              //接收起始位错误
		{
			SDIO_ClearFlag(SDIO_FLAG_STBITERR);                                   //清错误标志
			return SD_START_BIT_ERR;
		}

		taskEXIT_CRITICAL();                                                            //开启总中断
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                        //清除所有标记  
	}
	else if (DeviceMode == SD_DMA_MODE)
	{
		TransferError = SD_OK;
		StopCondition = 0;			                                            //单块写,不需要发送停止传输指令 
		TransferEnd = 0;				                                        //传输结束标置位，在中断服务置1
		SDIO->MASK |= (1 << 1) | (1 << 3) | (1 << 8) | (1 << 4) | (1 << 9);	    //配置产生数据接收完成中断
		SD_DMA_Config((u32*)buf, blksize, DMA_DIR_MemoryToPeripheral);			//SDIO DMA配置
		SDIO->DCTRL |= 1 << 3;								                    //SDIO DMA使能.  
		while (((DMA2->LISR&(1 << 27)) == RESET) && timeout)timeout--;          //等待传输完成 
		if (timeout == 0)
		{
			SD_Init();	 					                                    //重新初始化SD卡,可以解决写入死机的问题
			return SD_DATA_TIMEOUT;			                                    //超时	 
		}
		timeout = SDIO_DATATIMEOUT;
		while ((TransferEnd == 0) && (TransferError == SD_OK) && timeout)timeout--;
		if (timeout == 0)
			return SD_DATA_TIMEOUT;			                                    //超时	 
		if (TransferError != SD_OK)
			return TransferError;
	}
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                         //清除所有标记
	errorstatus = IsCardProgramming(&cardstate);
	while ((errorstatus == SD_OK) && ((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)))
	{
		errorstatus = IsCardProgramming(&cardstate);
	}
	return errorstatus;
}

/**************************************************
 *  功能:SD卡多块写入
 *  日期:2017/12/25 15:43
 *  输入:buf:数据缓存区 addr:写地址 nblks:要写入的块数
 *  输出:nblks:要写入的块数
 *  注意点:
 *************************************************/											   
SD_Error SD_WriteMultiBlocks(u8 *buf, long long addr, u16 blksize, u32 nblks)
{
	SD_Error errorstatus = SD_OK;
	u8  power = 0, cardstate = 0;
	u32 timeout = 0, bytestransferred = 0;
	u32 count = 0, restwords = 0;
	u32 tlen = nblks*blksize;				                                 //总长度(字节)
	u32 *tempbuff = (u32*)buf;
	if (buf == NULL)
		return SD_INVALID_PARAMETER;        
	SDIO->DCTRL = 0x0;							                            //数据控制寄存器清零(关DMA)   

	SDIO_DataInitStructure.SDIO_DataBlockSize = 0; ;	                    //清除DPSM状态机配置	
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	if (SDIO->RESP1&SD_CARD_LOCKED)
		return SD_LOCK_UNLOCK_FAILED;                                       //卡锁了
	if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)                             //大容量卡
	{
		blksize = 512;
		addr >>= 9;
	}
	if ((blksize > 0) && (blksize <= 2048) && ((blksize&(blksize - 1)) == 0))
	{
		power = convert_from_bytes_to_power_of_two(blksize);

		//发送CMD16+设置数据长度为blksize,短响应
		SDIO_CmdInitStructure.SDIO_Argument = blksize;	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);	               //等待R1响应  

		if (errorstatus != SD_OK)
			return errorstatus;   		 

	}
	else 
		return SD_INVALID_PARAMETER;
	if (nblks > 1)
	{
		if (nblks*blksize > SD_MAX_DATA_LENGTH)
			return SD_INVALID_PARAMETER;
		if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
		{
			//提高性能
			SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA << 16;		   //发送ACMD55,短响应 	
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_APP_CMD);		          //等待R1响应 

			if (errorstatus != SD_OK)
				return errorstatus;

			SDIO_CmdInitStructure.SDIO_Argument = nblks;		           //发送ACMD23,预擦除数据块,短响应 仅SD卡有效	 
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);          //等待R1响应 

			if (errorstatus != SD_OK)
				return errorstatus;

		}

		SDIO_CmdInitStructure.SDIO_Argument = addr;	                      //发送CMD25,多块写指令,短响应 	  
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);	          //等待R1响应   		   

		if (errorstatus != SD_OK)
			return errorstatus;

		SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4; ;	      //blksize, 控制器到卡	
		SDIO_DataInitStructure.SDIO_DataLength = nblks*blksize;
		SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
		SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
		SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
		SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
		SDIO_DataConfig(&SDIO_DataInitStructure);

		if (DeviceMode == SD_POLLING_MODE)
		{
			timeout = SDIO_DATATIMEOUT;
			taskENTER_CRITICAL();                                                            //关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
			while (!(SDIO->STA&((1 << 4) | (1 << 1) | (1 << 8) | (1 << 3) | (1 << 9))))//下溢/CRC/数据结束/超时/起始位错误
			{
				if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)					//发送区半空,表示至少存了8字(32字节)
				{
					if ((tlen - bytestransferred) < SD_HALFFIFOBYTES)                   //不够32字节了
					{
						restwords = ((tlen - bytestransferred) % 4 == 0) ? ((tlen - bytestransferred) / 4) : ((tlen - bytestransferred) / 4 + 1);
						for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
						{
							SDIO->FIFO = *tempbuff;
						}
					}
					else 									 	                        //发送区半空,可以发送至少8字(32字节)数据
					{
						for (count = 0; count < SD_HALFFIFO; count++)
						{
							SDIO->FIFO = *(tempbuff + count);
						}
						tempbuff += SD_HALFFIFO;
						bytestransferred += SD_HALFFIFOBYTES;
					}
					timeout = 0X3FFFFFFF;	                                            //写数据溢出时间
				}
				else
				{
					if (timeout == 0)
						return SD_DATA_TIMEOUT;
					timeout--;
				}
			}
			if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		               //数据超时错误
			{
				SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 	                               //清错误标志
				return SD_DATA_TIMEOUT;
			}
			else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	                //数据块CRC错误
			{
				SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		                            //清错误标志
				return SD_DATA_CRC_FAIL;
			}
			else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	                //接收fifo下溢错误
			{
				SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);		                                 //清错误标志
				return SD_TX_UNDERRUN;
			}
			else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	                //接收起始位错误
			{
				SDIO_ClearFlag(SDIO_FLAG_STBITERR);                                      //清错误标志
				return SD_START_BIT_ERR;
			}

			if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)		                     //发送结束
			{
				if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
				{
					SDIO_CmdInitStructure.SDIO_Argument = 0;                              //发送CMD12+结束传输 	  
					SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
					SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
					SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
					SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
					SDIO_SendCommand(&SDIO_CmdInitStructure);

					errorstatus = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);                  //等待R1响应   
					if (errorstatus != SD_OK)
						return errorstatus;
				}
			}
			taskEXIT_CRITICAL();                                                                  //开启总中断
			SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                              //清除所有标记
		}
		else if (DeviceMode == SD_DMA_MODE)
		{
			TransferError = SD_OK;
			StopCondition = 1;			                                                   //多块写,需要发送停止传输指令 
			TransferEnd = 0;				                                               //传输结束标置位，在中断服务置1
			SDIO->MASK |= (1 << 1) | (1 << 3) | (1 << 8) | (1 << 4) | (1 << 9);	           //配置产生数据接收完成中断
			SD_DMA_Config((u32*)buf, nblks*blksize, DMA_DIR_MemoryToPeripheral);		    //SDIO DMA配置
			SDIO->DCTRL |= 1 << 3;								                            //SDIO DMA使能. 
			timeout = SDIO_DATATIMEOUT;
			while (((DMA2->LISR&(1 << 27)) == RESET) && timeout)
				timeout--;                                                                  //等待传输完成 
			if (timeout == 0)	 								                            //超时
			{
				SD_Init();	 					                                            //重新初始化SD卡,可以解决写入死机的问题
				return SD_DATA_TIMEOUT;			                                            //超时	 
			}
			timeout = SDIO_DATATIMEOUT;
			while ((TransferEnd == 0) && (TransferError == SD_OK) && timeout)
				timeout--;
			if (timeout == 0)return SD_DATA_TIMEOUT;			                             //超时	 
			if (TransferError != SD_OK)
				return TransferError;
		}
	}
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                                      //清除所有标记
	errorstatus = IsCardProgramming(&cardstate);
	while ((errorstatus == SD_OK) && ((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)))
	{
		errorstatus = IsCardProgramming(&cardstate);
	}
	return errorstatus;
}
//SDIO中断服务函数		  
void SDIO_IRQHandler(void)
{
	SD_ProcessIRQSrc();//处理所有SDIO相关中断
}
//SDIO中断处理函数
//处理SDIO传输过程中的各种中断事务
//返回值:错误代码
SD_Error SD_ProcessIRQSrc(void)
{
	if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)//接收完成中断
	{
		if (StopCondition == 1)
		{
			SDIO_CmdInitStructure.SDIO_Argument = 0;//发送CMD12+结束传输 	  
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			TransferError = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);
		}
		else TransferError = SD_OK;
		SDIO->ICR |= 1 << 8;//清除完成中断标记
		SDIO->MASK &= ~((1 << 1) | (1 << 3) | (1 << 8) | (1 << 14) | (1 << 15) | (1 << 4) | (1 << 5) | (1 << 9));//关闭相关中断
		TransferEnd = 1;
		return(TransferError);
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)//数据CRC错误
	{
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		//清错误标志
		SDIO->MASK &= ~((1 << 1) | (1 << 3) | (1 << 8) | (1 << 14) | (1 << 15) | (1 << 4) | (1 << 5) | (1 << 9));//关闭相关中断
		TransferError = SD_DATA_CRC_FAIL;
		return(SD_DATA_CRC_FAIL);
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)//数据超时错误
	{
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);  			//清中断标志
		SDIO->MASK &= ~((1 << 1) | (1 << 3) | (1 << 8) | (1 << 14) | (1 << 15) | (1 << 4) | (1 << 5) | (1 << 9));//关闭相关中断
		TransferError = SD_DATA_TIMEOUT;
		return(SD_DATA_TIMEOUT);
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)//FIFO上溢错误
	{
		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);  			//清中断标志
		SDIO->MASK &= ~((1 << 1) | (1 << 3) | (1 << 8) | (1 << 14) | (1 << 15) | (1 << 4) | (1 << 5) | (1 << 9));//关闭相关中断
		TransferError = SD_RX_OVERRUN;
		return(SD_RX_OVERRUN);
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET)//FIFO下溢错误
	{
		SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);  			//清中断标志
		SDIO->MASK &= ~((1 << 1) | (1 << 3) | (1 << 8) | (1 << 14) | (1 << 15) | (1 << 4) | (1 << 5) | (1 << 9));//关闭相关中断
		TransferError = SD_TX_UNDERRUN;
		return(SD_TX_UNDERRUN);
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)//起始位错误
	{
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);  		//清中断标志
		SDIO->MASK &= ~((1 << 1) | (1 << 3) | (1 << 8) | (1 << 14) | (1 << 15) | (1 << 4) | (1 << 5) | (1 << 9));//关闭相关中断
		TransferError = SD_START_BIT_ERR;
		return(SD_START_BIT_ERR);
	}
	return(SD_OK);
}

/**************************************************
 *  功能:检查CMD0执行状态
 *  日期:2017/12/24 22:46
 *  输入:无
 *  输出:SD卡错误码，0，表示无错误
 *  注意点:
 *************************************************/
SD_Error CmdError(void)
{
	SD_Error errorstatus = SD_OK;
	u32 timeout = SDIO_CMD0TIMEOUT;
	while (timeout--)
	{
		if (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) != RESET)                   //命令已发送(无需响应)
			break;		 
	}
	if (timeout == 0)
		return SD_CMD_RSP_TIMEOUT;
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                        //清除所有标记
	return errorstatus;
}

/**************************************************
 *  功能:检查R7响应是否出错
 *  日期:2017/12/24 22:55
 *  输入:无
 *  输出:SD卡错误码
 *  注意点:
 *************************************************/
SD_Error CmdResp7Error(void)
{
	SD_Error errorstatus = SD_OK;
	u32 status;
	u32 timeout = SDIO_CMD0TIMEOUT;
	while (timeout--)
	{
		status = SDIO->STA;
		if (status&((1 << 0) | (1 << 2) | (1 << 6)))				//状态寄存器bit0:已接收命令响应（CRC校验失败）；bit2：命令响应超时（超时为64个卡时钟周期）；bit6：已接收命令响应（CRC通过）
			break;
	}
	if ((timeout == 0) || (status&(1 << 2)))	                   //响应超时
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;	                       //当前卡不是2.0兼容卡,或者不支持设定的电压范围
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT); 			           //清除命令响应超时标志
		return errorstatus;
	}
	if (status & (1 << 6))						                   //成功接收到响应
	{
		errorstatus = SD_OK;
		SDIO_ClearFlag(SDIO_FLAG_CMDREND); 				           //清除响应标志
	}
	return errorstatus;
}


/**************************************************
 *  功能:检查R1响应的错误状态
 *  日期:2017/12/24 23:22
 *  输入:命令索引号
 *  输出:错误码，0表示无错误
 *  注意点:
 *************************************************/
SD_Error CmdResp1Error(u8 cmd)
{
	u32 status;
	while (1)
	{
		status = SDIO->STA;
		if (status&((1 << 0) | (1 << 2) | (1 << 6)))
			break;
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT) != RESET)					//bit3
	{
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT); 				                //清除命令响应超时标志
		return SD_CMD_RSP_TIMEOUT;
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL) != RESET)					//CRC错误bit0
	{
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL); 				                //清除标志
		return SD_CMD_CRC_FAIL;
	}
	if (SDIO->RESPCMD != cmd)								                //命令不匹配 （R1响应【45:40】位命令索引）
		return SD_ILLEGAL_CMD;
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                      //清除所有标记
	return (SD_Error)(SDIO->RESP1&SD_OCR_ERRORBITS);                     
}

/**************************************************
 *  功能:检查R3响应的错误状态
 *  日期:2017/12/24 23:37
 *  输入:无
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error CmdResp3Error(void)
{
	u32 status;
	while (1)
	{
		status = SDIO->STA;
		if (status&((1 << 0) | (1 << 2) | (1 << 6)))                         //CRC错误/命令响应超时/已经收到响应(CRC校验成功)
			break;	
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT) != RESET)					 //响应超时
	{
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);			                         //清除命令响应超时标志
		return SD_CMD_RSP_TIMEOUT;
	}
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                       //清除所有标记
	return SD_OK;
}

/**************************************************
 *  功能:检查R2响应的错误状态
 *  日期:2017/12/25 0:01
 *  输入:无
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error CmdResp2Error(void)
{
	SD_Error errorstatus = SD_OK;
	u32 status;
	u32 timeout = SDIO_CMD0TIMEOUT;
	while (timeout--)
	{
		status = SDIO->STA;
		if (status&((1 << 0) | (1 << 2) | (1 << 6)))
			break;
	}
	if ((timeout == 0) || (status&(1 << 2)))	                              //响应超时
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT); 		                          //清除命令响应超时标志
		return errorstatus;
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL) != RESET)					  //CRC错误
	{
		errorstatus = SD_CMD_CRC_FAIL;
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);		                              //清除标志
	}
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                        //清除所有标记
	return errorstatus;
}

/**************************************************
 *  功能:检查R6响应的错误状态
 *  日期:2017/12/25 0:07
 *  输入:命令索引号，卡放回RCA地址
 *  输出:错误状态（间接放回RCA）
 *  注意点:
 *************************************************/
SD_Error CmdResp6Error(u8 cmd, u16*prca)
{
	SD_Error errorstatus = SD_OK;
	u32 status;
	u32 rspr1;
	while (1)
	{
		status = SDIO->STA;
		if (status&((1 << 0) | (1 << 2) | (1 << 6)))
			break;
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT) != RESET)					
	{
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);			
		return SD_CMD_RSP_TIMEOUT;
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL) != RESET)						
	{
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);				
		return SD_CMD_CRC_FAIL;
	}
	if (SDIO->RESPCMD != cmd)				                 //匹配cmd命令
	{
		return SD_ILLEGAL_CMD;
	}
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                       //清除所有标记
	rspr1 = SDIO->RESP1;					                 //得到响应 	 
	if (SD_ALLZERO == (rspr1&(SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
	{
		*prca = (u16)(rspr1 >> 16);			                 //右移16位得到,rca
		return errorstatus;
	}
	if (rspr1&SD_R6_GENERAL_UNKNOWN_ERROR)
		return SD_GENERAL_UNKNOWN_ERROR;
	if (rspr1&SD_R6_ILLEGAL_CMD)
		return SD_ILLEGAL_CMD;
	if (rspr1&SD_R6_COM_CRC_FAILED)
		return SD_COM_CRC_FAILED;
	return errorstatus;
}


/**************************************************
 *  功能:SDIO使能总线宽模式
 *  日期:2017/12/25 9:54
 *  输入:总线模式
 *  输出:错误码
 *  注意点:
 *************************************************/
SD_Error SDEnWideBus(u8 enx)
{
	SD_Error errorstatus = SD_OK;
	u32 scr[2] = { 0,0 };
	u8 arg = 0X00;											                      //ACMD6参数
	if (enx)
		arg = 0X02;
	else
		arg = 0X00;
	if (SDIO->RESP1&SD_CARD_LOCKED)												 //SD卡处于LOCKED状态	
		return SD_LOCK_UNLOCK_FAILED;	    
	errorstatus = FindSCR(RCA, scr);											 //得到SCR寄存器数据和设置线宽
	if (errorstatus != SD_OK)
		return errorstatus;
	if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)							 //支持宽总线
	{
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)RCA << 16;               //发送CMD55+RCA,短响应	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

		if (errorstatus != SD_OK)
			return errorstatus;

		SDIO_CmdInitStructure.SDIO_Argument = arg;                              //发送ACMD6,短响应,参数:10,4位;00,1位.	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

		return errorstatus;
	}
	else																		//不支持宽总线设置
		return SD_REQUEST_NOT_APPLICABLE;				 	 
}
//检查卡是否正在执行写操作
//pstatus:当前状态.
//返回值:错误代码
SD_Error IsCardProgramming(u8 *pstatus)
{
	vu32 respR1 = 0, status = 0;

	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)RCA << 16; //卡相对地址参数
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;//发送CMD13 	
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	status = SDIO->STA;

	while (!(status&((1 << 0) | (1 << 6) | (1 << 2))))status = SDIO->STA;//等待操作完成
	if (SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL) != RESET)			//CRC检测失败
	{
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);	//清除错误标记
		return SD_CMD_CRC_FAIL;
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT) != RESET)			//命令超时 
	{
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);			//清除错误标记
		return SD_CMD_RSP_TIMEOUT;
	}
	if (SDIO->RESPCMD != SD_CMD_SEND_STATUS)return SD_ILLEGAL_CMD;
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记
	respR1 = SDIO->RESP1;
	*pstatus = (u8)((respR1 >> 9) & 0x0000000F);
	return SD_OK;
}
//读取当前卡状态
//pcardstatus:卡状态
//返回值:错误代码
SD_Error SD_SendStatus(uint32_t *pcardstatus)
{
	SD_Error errorstatus = SD_OK;
	if (pcardstatus == NULL)
	{
		errorstatus = SD_INVALID_PARAMETER;
		return errorstatus;
	}

	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)RCA << 16;//发送CMD13,短响应		 
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);	//查询响应状态 
	if (errorstatus != SD_OK)return errorstatus;
	*pcardstatus = SDIO->RESP1;//读取响应值
	return errorstatus;
}
//返回SD卡的状态
//返回值:SD卡状态
SDCardState SD_GetState(void)
{
	u32 resp1 = 0;
	if (SD_SendStatus(&resp1) != SD_OK)return SD_CARD_ERROR;
	else return (SDCardState)((resp1 >> 9) & 0x0F);
}

/**************************************************
 *  功能:查找SD卡SCR寄存器值
 *  日期:2017/12/25 10:02
 *  输入:rca:卡相对地址；pscr:数据缓存区(存储SCR内容64bit)
 *  输出:错误码
 *  注意点:
 *************************************************/	   
SD_Error FindSCR(u16 rca, u32 *pscr)
{
	u32 index = 0;
	SD_Error errorstatus = SD_OK;
	u32 tempscr[2] = { 0,0 };

	/*发送CMD16,短响应，设置SD卡块大小，单位为字节，设置后，后续所有的块操作都是以此大小位基准；对高容量SD卡块大小固定512字节，不受此指令影响	*/
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;	                  
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;                  //cmd16
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;                  //r1
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

	if (errorstatus != SD_OK)
		return errorstatus;

	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)RCA << 16;					//选取SD卡
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;                       //CMD55,短响应，在所有APPCMD之前都应先发此指令
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;					//r1
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
	if (errorstatus != SD_OK)
		return errorstatus;

	/*配置SDIO数据传输模式*/
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 8;                                  //8个字节长度,block为8字节,.
	SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b;           //块大小8byte 
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;			 //SD卡到SDIO		
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;		     //块传输模式
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;					     //数据状态机都是使能
	SDIO_DataConfig(&SDIO_DataInitStructure);

	SDIO_CmdInitStructure.SDIO_Argument = 0x0;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR;	              //发送ACMD51,短响应,读取SD卡配置寄存器SCR
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;                    //r1
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);
	if (errorstatus != SD_OK)
		return errorstatus;
	while (!(SDIO->STA&(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
	{
		if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)                     //接收FIFO数据可用
		{
			*(tempscr + index) = SDIO->FIFO;								   //读取FIFO内容
			index++;
			if (index >= 2)
				break;
		}
	}
	if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)					  //数据超时错误
	{
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 							      //清错误标志
		return SD_DATA_TIMEOUT;
	}
	else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	              //数据块CRC错误
	{
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		                          //清错误标志
		return SD_DATA_CRC_FAIL;
	}
	else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) 	              //接收fifo上溢错误
	{
		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);		                             //清错误标志
		return SD_RX_OVERRUN;
	}
	else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	             //接收起始位错误
	{
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);                                  //清错误标志
		return SD_START_BIT_ERR;
	}
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);                                       //清除所有标记

//把数据顺序按8位为单位倒过来.   	
	*(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);
	*(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);
	return errorstatus;
}

/**************************************************
 *  功能:得到NumberOfBytes以2为底的指数
 *  日期:2017/12/25 10:57
 *  输入:NumberOfBytes:字节数.
 *  输出:以2为底的指数值
 *  注意点:
 *************************************************/
u8 convert_from_bytes_to_power_of_two(u16 NumberOfBytes)
{
	u8 count = 0;
	while (NumberOfBytes != 1)
	{
		NumberOfBytes >>= 1;
		count++;
	}
	return count;
}

//配置SDIO DMA  
//mbuf:存储器地址
//bufsize:传输数据量
//dir:方向;DMA_DIR_MemoryToPeripheral  存储器-->SDIO(写数据);DMA_DIR_PeripheralToMemory SDIO-->存储器(读数据);
void SD_DMA_Config(u32*mbuf, u32 bufsize, u32 dir)
{

	DMA_InitTypeDef  DMA_InitStructure;

	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE) {}//等待DMA可配置 

	DMA_DeInit(DMA2_Stream3);//清空之前该stream3上的所有中断标志


	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SDIO->FIFO;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)mbuf;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = dir;//存储器到外设模式
	DMA_InitStructure.DMA_BufferSize = 0;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//外设数据长度:32位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;//存储器数据长度:32位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;   //FIFO使能      
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//全FIFO
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;//外设突发4次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;//存储器突发4次传输
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);//初始化DMA Stream

	DMA_FlowControllerConfig(DMA2_Stream3, DMA_FlowCtrl_Peripheral);//外设流控制 

	DMA_Cmd(DMA2_Stream3, ENABLE);//开启DMA传输	 

}


//读SD卡
//buf:读数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;				  				 
u8 SD_ReadDisk(u8*buf, u32 sector, u8 cnt)
{
	u8 sta = SD_OK;
	long long lsector = sector;
	u8 n;
	lsector <<= 9;
	if ((u32)buf % 4 != 0)
	{
		for (n = 0; n < cnt; n++)
		{
			sta = SD_ReadBlock(SDIO_DATA_BUFFER, lsector + 512 * n, 512);//单个sector的读操作
			memcpy(buf, SDIO_DATA_BUFFER, 512);
			buf += 512;
		}
	}
	else
	{
		if (cnt == 1)sta = SD_ReadBlock(buf, lsector, 512);    	//单个sector的读操作
		else sta = SD_ReadMultiBlocks(buf, lsector, 512, cnt);//多个sector  
	}
	return sta;
}
//写SD卡
//buf:写数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;	
u8 SD_WriteDisk(u8*buf, u32 sector, u8 cnt)
{
	u8 sta = SD_OK;
	u8 n;
	long long lsector = sector;
	lsector <<= 9;
	if ((u32)buf % 4 != 0)
	{
		for (n = 0; n < cnt; n++)
		{
			memcpy(SDIO_DATA_BUFFER, buf, 512);
			sta = SD_WriteBlock(SDIO_DATA_BUFFER, lsector + 512 * n, 512);//单个sector的写操作
			buf += 512;
		}
	}
	else
	{
		if (cnt == 1)sta = SD_WriteBlock(buf, lsector, 512);    	//单个sector的写操作
		else sta = SD_WriteMultiBlocks(buf, lsector, 512, cnt);	//多个sector  
	}
	return sta;
}

/**************************************************
 *  功能:T通过串口打印SD卡相关信息
 *  日期:2017/12/25 18:38
 *  输入:无
 *  输出:无
 *  注意点:
 *************************************************/
void show_sdcard_info(void)
{
	switch (SDCardInfo.CardType)
	{
	case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("\r\nCard Type:SDSC V1.1\r\n"); 
		break;
	case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("\r\nCard Type:SDSC V2.0\r\n");
		break;
	case SDIO_HIGH_CAPACITY_SD_CARD:printf("\r\nCard Type:SDHC V2.0\r\n"); 
		break;
	case SDIO_MULTIMEDIA_CARD:printf("\r\nCard Type:MMC Card\r\n"); 
		break;
	}
	printf("Card ManufacturerID:%d\r\n", SDCardInfo.SD_cid.ManufacturerID);	     //制造商ID
	printf("Card RCA:%d\r\n", SDCardInfo.RCA);								     //卡相对地址
	printf("Card Capacity:%d MB\r\n", (u32)(SDCardInfo.CardCapacity >> 20));	//显示容量
	printf("Card BlockSize:%d\r\n\r\n", SDCardInfo.CardBlockSize);			    //显示块大小
}







