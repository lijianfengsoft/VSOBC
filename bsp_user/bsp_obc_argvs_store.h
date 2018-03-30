#pragma once

#define OBC_STORE_ADDR		((uint32_t)(0x080C0000))

typedef struct __attribute__((packed))
{
	uint32_t    obc_boot_count;
	uint32_t    obc_reset_time;
	uint32_t 	antenna_status;
	uint32_t    hk_down_cnt;
	uint32_t    hk_store_cnt;

} obc_save_t;


uint8_t obc_argvs_recover(void);
uint8_t obc_argvs_store(void);