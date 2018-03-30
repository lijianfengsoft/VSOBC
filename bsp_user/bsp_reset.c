/***************************************************************************
*  功能：对CPU复位情况作出判断，并打印出复位时的信息,并其中包括一个检测任务是否溢出的钩子函数
*  版本：V 1.0 
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/11 17:31 
*****************************************************************************/                                                    

#include "stm32f4xx.h"
#include "core_cm4.h"

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

#include "driver_debug.h"

#include "bsp_obc_argvs_store.h"
#include "bsp_reset.h"

#define STR_RESET_NONE				"CPU_RESET_NONE"
#define STR_RESET_USER				"CPU_RESET_USER"
#define STR_RESET_STACK_OVERFLOW	"CPU_RESET_STACK_OVERFLOW"
#define STR_RESET_HardFault			"CPU_RESET_HardFault"
#define STR_RESET_MemManage			"CPU_RESET_MemManage"
#define STR_RESET_BusFault			"CPU_RESET_BusFault"
#define STR_RESET_UsageFault		"CPU_RESET_UsageFault"
#define STR_RESET_DebugMon			"CPU_RESET_DebugMon"
#define STR_RESET_NMI				"CPU_RESET_NMI"

unsigned int __attribute__((section(".ram_persist"))) cpu_reset_cause;
unsigned int __attribute__((section(".ram_persist"))) lr;
char __attribute__((section(".ram_persist"))) TaskName[configMAX_TASK_NAME_LEN];

/**************************************************
 *  功能:复位函数，保存复位前的数据
 *  日期:2018/1/11 18:02
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void cpu_reset(void)
{
	if (xTaskGetCurrentTaskName(TaskName))
		xTaskGetCurrentTaskName(TaskName);

	reset_cause_print(cpu_reset_cause);

	obc_argvs_store();

	NVIC_SystemReset();							//自复位，主要是寄存器SCB->AIRCR第二位置一就发生逻辑的复位
}

/**************************************************
 *  功能:直接复位，不保存数据
 *  日期:2018/1/11 19:54
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void reset(void)
{
	NVIC_SystemReset();
}

/**************************************************
 *  功能:打印各种复位情况的信息
 *  日期:2018/1/11 19:54
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void reset_cause_print(int cause)
{
	switch (cause) {
	case CPU_RESET_NONE:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_NONE, TaskName, lr);
		break;
	case CPU_RESET_USER:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_USER, TaskName, lr); 
		break;
	case CPU_RESET_STACK_OVERFLOW:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_STACK_OVERFLOW, TaskName, lr); 
		break;
	case CPU_RESET_HardFault:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_HardFault, TaskName, lr); 
		break;
	case CPU_RESET_MemManage:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_MemManage, TaskName, lr); 
		break;
	case CPU_RESET_BusFault:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_BusFault, TaskName, lr); 
		break;
	case CPU_RESET_UsageFault:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_UsageFault, TaskName, lr); 
		break;
	case CPU_RESET_DebugMon:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_DebugMon, TaskName, lr); 
		break;
	case CPU_RESET_NMI:
		printf("RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_NMI, TaskName, lr); 
		break;
	default:
		break;
	}
}

/**************************************************
 *  功能:获取lr的值
 *  日期:2018/1/11 19:57
 *  输入:无
 *  输出:当前LR（r14）的值
 *  注意点:最后-4是为了获取前一个指令时的LR
 *************************************************/
unsigned int get_banked_lr(void)
{
	unsigned int ip;

	__asm volatile ("mov %[c], lr" : [c]"=r"(ip));

	return ip - 4;
}

/**************************************************
 *  功能:获取造成复位的原因
 *  日期:2018/1/11 20:01
 *  输入:造成复位的枚举
 *  输出:
 *  注意点:将原因赋给全局变量cause
 *************************************************/
void cpu_set_reset_cause(cpu_reset_cause_t cause) 
{
	cpu_reset_cause = cause;
}

/**************************************************
 *  功能:接下来是各种错误中断的处理
 *  日期:2018/1/11 20:04
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void NMI_Handler(void) __attribute__((naked));
void NMI_Handler(void)
{
	lr = get_banked_lr();
	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_NMI);

	cpu_reset();
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
	lr = get_banked_lr();
	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_HardFault);

	cpu_reset();
}

void MemManage_Handler(void) __attribute__((naked));
void MemManage_Handler(void)
{
	lr = get_banked_lr();
	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_MemManage);

	cpu_reset();
}

void BusFault_Handler(void) __attribute__((naked));
void BusFault_Handler(void)
{
	lr = get_banked_lr();
	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_BusFault);

	cpu_reset();
}

void UsageFault_Handler(void) __attribute__((naked));
void UsageFault_Handler(void)
{
	lr = get_banked_lr();
	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_UsageFault);

	cpu_reset();
}

void DebugMon_Handler(void) __attribute__((naked));
void DebugMon_Handler(void)
{
	lr = get_banked_lr();
	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_DebugMon);

	cpu_reset();
}

/**************************************************
 *  功能:检测任务是否溢出的钩子函数
 *  日期:2018/1/11 20:25
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {

	printf("STACK OVERFLOW!\r\n");
	printf("In task %p name: %s\r\n", pxTask, pcTaskName);
	lr = get_banked_lr();
	printf("NMI INTERRUPT: LR=%#0x\r\n", lr);

	volatile unsigned int i = 0xFFFF;
	while (i--);
	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_STACK_OVERFLOW);
	cpu_reset();

}