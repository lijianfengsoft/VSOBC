#pragma once
#include <sys/stat.h>

#define  MAX_USART_BUFFSIZE 1024
#define CONSOLE_LENGTH 0x2000		//队列块数

struct USART1_TypeDefStruct
{
	uint8_t USART_Read_buf[MAX_USART_BUFFSIZE];
	uint16_t USART_Read_Head;
	uint16_t USART_Read_Tail;
	uint8_t USART_Write_buf[MAX_USART_BUFFSIZE];
	uint16_t USART_Write_Head;
	uint16_t USART_Write_Tail;
};

void usart_init(uint32_t bound);
void Console_ReceivedByte(uint8_t R_data);
void usart_confing(struct USART1_TypeDefStruct *pUSART);
void USART1_IRQ(struct USART1_TypeDefStruct *pUSART);
uint8_t Usart_read(struct USART1_TypeDefStruct *pUSART);
void Usart_write(struct USART1_TypeDefStruct *pUSART, uint8_t senddata);
void Usart_writes(struct USART1_TypeDefStruct *pUSART, uint8_t *sendbuff);
char * strdup(const char *s);
size_t  strnlen(const char *str, size_t maxsize);

int _fstat(int fd, struct stat *pStat);
int _isatty(int fd);
int _lseek(int a, int b, int c);
int _read(int fd, char *pBuffer, int size);
caddr_t _sbrk(int increment);