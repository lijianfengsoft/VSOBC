#pragma once
#include "stm32f4xx.h"
#define EXT_SRAM_ADDR  	((uint32_t)0x68000000)
#define EXT_SRAM_SIZE	(2 * 1024 * 1024)

void FSMC_SRAM_ReadBuffer(u8* pBuffer, u32 ReadAddr, u32 n);
void FSMC_SRAM_WriteBuffer(u8* pBuffer, u32 WriteAddr, u32 n);
void EXT_SRAMInit(void);