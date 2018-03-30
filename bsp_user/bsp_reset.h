#pragma once


typedef enum {
	CPU_RESET_NONE = 0,
	CPU_RESET_USER = 1,
	CPU_RESET_STACK_OVERFLOW,
	CPU_RESET_HardFault,
	CPU_RESET_MemManage,
	CPU_RESET_BusFault,
	CPU_RESET_UsageFault,
	CPU_RESET_DebugMon,
	CPU_RESET_NMI,
} cpu_reset_cause_t;

extern unsigned int __attribute__((section(".ram_persist"))) cpu_reset_cause;
extern unsigned int __attribute__((section(".ram_persist"))) lr;
extern char __attribute__((section(".ram_persist"))) TaskName[configMAX_TASK_NAME_LEN];

void cpu_reset(void);
void reset(void);
unsigned int get_banked_lr(void);
void cpu_set_reset_cause(cpu_reset_cause_t cause);
