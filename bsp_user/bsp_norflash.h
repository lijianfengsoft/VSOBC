#pragma once
/*
制造商ID: Spansion   0x01
硬件读出来的ID为NorFlash ID = 0x1, 0x2253, 0x220a, 0x2200
*/
typedef enum
{
	NOR_SUCCESS = 0,
	NOR_ONGOING = 1,
	NOR_ERROR = 2,
	NOR_TIMEOUT = 3
}NOR_Status;

void Norflash_Init(void);
void NOR_ReadID(void);
NOR_Status NOR_EraseBlock(uint32_t BlockAddr);
NOR_Status NOR_EraseChip(void);
NOR_Status Nor_Reset(void);
NOR_Status NOR_ReturnToReadMode(void);
NOR_Status NOR_WriteHalfWord(u32 WriteAddr, u16 Data);
u16 NOR_ReadHalfWord(u32 ReadAddr);
NOR_Status NOR_WriteBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);
NOR_Status NOR_ProgramBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);
void NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead);
NOR_Status NOR_GetStatus(u32 Timeout);