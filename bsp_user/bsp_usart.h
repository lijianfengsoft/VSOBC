#pragma once

extern uint8_t Picturedata[500 * 1024];

void bsp_usart_init(void);
void USART3_DMA_Config(void);
