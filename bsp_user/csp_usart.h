#pragma once


/**
* In order to catch incoming chars use the callback.
* Only one callback per interface.
* @param handle usart[0,1,2,3]
* @param callback function pointer
*/
typedef void(*usart_callback_t) (uint8_t *buf, int len, void *pxTaskWoken);
void usart_set_callback(usart_callback_t callback);

/**
* Insert a character to the RX buffer of a usart
* @param handle usart[0,1,2,3]
* @param c Character to insert
*/
void usart_insert(char c, void *pxTaskWoken);

/**
* Polling putchar
*
* @param handle usart[0,1,2,3]
* @param c Character to transmit
*/
void usart3_putc(char c);

void csp_usart3_init(uint32_t bound);

CSP_DEFINE_TASK(task_client);