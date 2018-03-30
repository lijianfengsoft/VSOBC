/***************************************************************************
*  功能：初始化ADC1的通道16来获取CPU内部温度传感器的值
*  版本：V1.0
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/6 22:54 
*****************************************************************************/                                                    
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"
#include "bsp_intemp.h"
#include "FreeRTOS.h"

/**************************************************
 *  功能:初始化ADC1的16通道
 *  日期:2018/1/6 23:09
 *  输入:无
 *  输出:无
 *  注意点:内部温度传感器需要选中
 *************************************************/
void intemp_adcinit(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_InitTypeDef ADC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit();
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_Cmd(ADC1,ENABLE);

	ADC_TempSensorVrefintCmd(ENABLE);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_16,1,ADC_SampleTime_480Cycles);

}

/**************************************************
 *  功能:读取和计算温度值
 *  日期:2018/1/6 23:13
 *  输入:无
 *  输出:温度值
 *  注意点:参考电压Vref要精确
 *************************************************/
float Get_Temprate(void)
{
	u32 adcx;
	float temperate;

	ADC_SoftwareStartConv(ADC1);
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	adcx= ADC_GetConversionValue(ADC1);
	temperate = (float)adcx*(2.5 / 4096);
	temperate = (temperate - 0.76) / 0.0025 + 25;

	return temperate;
}