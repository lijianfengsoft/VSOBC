#pragma once
#include "queue.h"

#define I2C_MTU     255

#define I2C_MASTER  0
#define I2C_SLAVE   1

#define MAX_DEVICES                     2
#define DEVICE_MODE_M_T                 0
#define DEVICE_MODE_M_R                 1

#define FSMC_Bank1_SRAM1_ADDR  ((uint32_t)0x60000000)
#define FSMC_Bank1_SRAM4_ADDR  ((uint32_t)0x6C000000)

typedef struct __attribute__((packed)) i2c_frame_s {
	uint8_t padding;
	uint8_t retries;
	uint32_t reserved;
	uint8_t dest;
	uint8_t len_rx;
	uint16_t len;
	uint8_t data[I2C_MTU];
} i2c_frame_t;

typedef void(*i2c_callback_t) (i2c_frame_t * frame, void * pxTaskWoken);

typedef struct pca9665_transmission_object_s {
	xQueueHandle queue;
	i2c_frame_t * frame;
	uint16_t next_byte;
} pca9665_transmission_object_t;

typedef struct pca9665_device_object_s {
	uint8_t * base;
	pca9665_transmission_object_t rx;
	pca9665_transmission_object_t tx;
	uint16_t speed;
	volatile unsigned int is_initialised;
	volatile unsigned int slave_addr;
	volatile unsigned int is_busy;
	volatile unsigned int mode;
	i2c_callback_t callback;
} pca9665_device_object_t;

/** Registers */
#define I2CSTA							0
#define INDPTR							0
#define I2CDAT							1
#define INDIRECT						2
#define I2CCON							3
#define I2CCOUNT						(4+0)
#define I2CADR							(4+1)
#define I2CSCLL							(4+2)
#define I2CSCLH							(4+3)
#define I2CTO							(4+4)
#define I2CPRESET						(4+5)
#define I2CMODE							(4+6)

/** Control register bits */
#define CON_AA 							0x80
#define CON_ENSIO 						0x40
#define CON_STA 						0x20
#define CON_STO 						0x10
#define CON_SI							0x08
#define CON_MODE 						0x01

/** Status register bits */
#define STA_IDLE						0xF8
#define STA_M_START_SENDT				0x08
#define STA_M_REPEATED_START_SENDT		0x10
#define STA_M_SLAW_SENDT_ACKED			0x18
#define STA_M_SLAW_SENDT_NACKED			0x20
#define STA_M_DATA_SENDT_ACKED			0x28
#define STA_M_DATA_SENDT_LAST_NACKED	0x30
#define STA_M_ARBITRATION_LOST			0x38

#define STA_M_SLAR_SENT_ACKED 			0x40
#define STA_M_SLAR_SENT_NACKED 			0x48
#define STA_M_DATA_RECEIVED_ACKED		0x50
#define STA_M_DATA_RECEIVED_NACKED		0x58

#define STA_S_SLAW_RECEIVED_ACKED		0x60
#define STA_S_ARB_LOST_SLAW_RECEIVED	0x68
#define STA_S_GC_RECEIVED				0xD0
#define STA_S_ARB_LOST_GC_RECEIVED		0xD8
#define STA_S_DATA_RECEIVED_SLA_ACKED	0x80
#define STA_S_DATA_RECEIVED_SLA_NACKED	0x88
#define STA_S_DATA_RECEIVED_GC_ACKED	0xE0
#define STA_S_DATA_RECEIVED_GC_NACKED	0xE8
#define STA_S_STOP_REP_RECEIVED			0xA0

#define PCA9665_MAX_BUF					68

unsigned int i2c_error_count;


int i2c_init(int handle, int mode, uint8_t addr, uint16_t speed, int queue_len_tx, int queue_len_rx, i2c_callback_t callback);
void PCA9665_IO_Init(void);
int i2c_receive(int handle, i2c_frame_t ** frame, uint16_t timeout);
int i2c_send(int handle, i2c_frame_t * frame, uint16_t timeout);
int i2c_master_transaction(int handle, uint8_t addr, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout);
void pca9665_dump_regs(int handler);
void __attribute__((noinline)) pca9665_dsr(portBASE_TYPE * task_woken);
void pca9665_isr_init(void);



