

#include "System.h"  

//进入待机模式	  
void Sys_Standby(void)
{
	SCB->SCR |= 1 << 2;		//使能SLEEPDEEP位 (SYS->CTRL)	   
	RCC->APB1ENR |= 1 << 28;//使能电源时钟 
	PWR->CSR |= 1 << 8;     //设置WKUP用于唤醒
	PWR->CR |= 1 << 2;      //清除Wake-up 标志
	PWR->CR |= 1 << 1;      //PDDS置位   	
	WFI_SET();			//执行WFI指令,进入待机模式		 
}
//系统软复位   
void Sys_Soft_Reset(void)
{
	SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;
}
//时钟设置函数
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCO频率
//Fsys:系统时钟频率
//Fusb:USB,SDIO,RNG等的时钟频率
//Fs:PLL输入时钟频率,可以是HSI,HSE等. 
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.

//外部晶振为8M的时候,推荐值:plln=336,pllm=8,pllp=2,pllq=7.
//得到:Fvco=8*(336/8)=336Mhz
//     Fsys=336/2=168Mhz
//     Fusb=336/7=48Mhz
//返回值:0,成功;1,失败。
uint8_t Sys_Clock_Set(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
	uint16_t retry = 0;
	uint8_t status = 0;
	RCC->CR |= 1 << 16;				//HSE 开启 
	while (((RCC->CR&(1 << 17)) == 0) && (retry < 0X1FFF))retry++;//等待HSE RDY
	if (retry == 0X1FFF)status = 1;	//HSE无法就绪
	else
	{
		RCC->APB1ENR |= 1 << 28;	//电源接口时钟使能
		PWR->CR |= 3 << 14; 		//高性能模式,时钟可到168Mhz
		RCC->CFGR |= (0 << 4) | (5 << 10) | (4 << 13);//HCLK 不分频;APB1 4分频;APB2 2分频. 
		RCC->CR &= ~(1 << 24);	//关闭主PLL
		RCC->PLLCFGR = pllm | (plln << 6) | (((pllp >> 1) - 1) << 16) | (pllq << 24) | (1 << 22);//配置主PLL,PLL时钟源来自HSE
		RCC->CR |= 1 << 24;			//打开主PLL
		while ((RCC->CR&(1 << 25)) == 0);//等待PLL准备好 
		FLASH->ACR |= 1 << 8;		//指令预取使能.
		FLASH->ACR |= 1 << 9;		//指令cache使能.
		FLASH->ACR |= 1 << 10;		//数据cache使能.
		FLASH->ACR |= 5 << 0;		//5个CPU等待周期. 
		RCC->CFGR &= ~(3 << 0);		//清零
		RCC->CFGR |= 2 << 0;		//选择主PLL作为系统时钟	 
		while ((RCC->CFGR&(3 << 2)) != (2 << 2));//等待主PLL作为系统时钟成功. 
	}
	return status;
}

//系统时钟初始化函数
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.
// 

void Stm32_Clock_Init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
	RCC->CR |= 0x00000001;		//设置HISON,开启内部高速RC振荡
	RCC->CFGR = 0x00000000;		//CFGR清零 
	RCC->CR &= 0xFEF6FFFF;		//HSEON,CSSON,PLLON清零 
	RCC->PLLCFGR = 0x24003010;	//PLLCFGR恢复复位值 
	RCC->CR &= ~(1 << 18);			//HSEBYP清零,外部晶振不旁路
	RCC->CIR = 0x00000000;		//禁止RCC时钟中断 
	Sys_Clock_Set(plln, pllm, pllp, pllq);//设置时钟 
										  //配置向量表				  
#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(1 << 29, 0x0);
#else   
	MY_NVIC_SetVectorTable(0, 0x0);
#endif 
}

//设置向量表偏移地址
//NVIC_VectTab:基址
//Offset:偏移量		 
void MY_NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{
	SCB->VTOR = NVIC_VectTab | (Offset&(uint32_t)0xFFFFFE00);//设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留。
}

////THUMB指令不支持汇编内联
////采用如下方法实现执行汇编指令WFI  
//__asm void WFI_SET(void)
//{
//	WFI;		  
//}
////关闭所有中断(但是不包括fault和NMI中断)
//__asm void INTX_DISABLE(void)
//{
//	CPSID   I
//	BX      LR	  
//}
////开启所有中断
//__asm void INTX_ENABLE(void)
//{
//	CPSIE   I
//	BX      LR 
//}
////设置栈顶地址
////addr:栈顶地址
//__asm void MSR_MSP(uint32_t addr) 
//{
//	MSR MSP, r0 			//set Main Stack value
//	BX r14
//}


