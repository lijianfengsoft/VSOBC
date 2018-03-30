#pragma once

#include <time.h>

#define DS1302_CMD_WRITE		(0 << 0)
#define DS1302_CMD_READ			(1 << 0)

#define DS1302_CH_MASK			(0x80)	//启动或暂停 Seconds bit7 0启动，1暂停
#define DS1302_12HR_MASK		(0x80)	//选择12小时制还是24小时制 Hour bit7 0为24小时制，1为12小时制
#define DS1302_PM_MASK			(0x20)	//12小时制模式下,位 5是上午/下午位且高电平是下午. 24小时制模式下, 位 5 是第二 10 - 小时位(20点–23点).
#define DS1302_WP_MASK			(0x80)	//控制寄存器的位 7是写保护位，1保护，0为禁止保护

/*配置涓流充电寄存器的值为0xA5（(位 4到 位 7)控制涓流充电器的选择.为了防止意外使能,只有 1010的模式才能使涓流充电器使能.
*位 2和位 3 选择 VCC2和 VCC1之间连了一个还是两个二极管;位 0 和位 1选择连在 VCC2 和 VCC1之间的电阻.*/
#define SET_TRICKLE_REGISTER	(0XA5)   

#define DS1302_CMD_TRICKLE_REG	(0X90)  //对涓流充电器寄存器操作的命令/地址字节（除读写位，其他命令都是除此位）
#define DS1302_CMD_SECONDS_REG	(0X80)  //Seconds bit7 CH时钟暂停标志,当此为置 1时，时钟振荡器暂停，低功耗备用模式;当此为置 0时,时钟开始.初始加电状态未定义 
										//bit4-bit6 10; bit0-bit3 00-59
#define DS1302_CMD_MINUTES_REG	(0X82)  //Minutes bit4-bit6:10; bit0-bit3  00-59
#define DS1302_CMD_HOUR_REG		(0X84)  //Hour bit7:高12 低24模式  bit5：（12模式）高PM，低AM ；（24模式）10进位  bit0-bit3：时刻   1-12/0-23
#define DS1302_CMD_DATE_REG		(0X86)  //Date  bit4-bit5 10; bit0-bit3 1-32
#define DS1302_CMD_MONTH_REG	(0X88)  //bit4 10;  bit0-bit3  1-12
#define DS1302_CMD_DAY_REG		(0X8A)  //DAY bit0-bit2 1-7
#define DS1302_CMD_YEAR_REG		(0X8C)  //year bit4-bit7 10 ; bit0-bit3  00-99
#define DS1302_CMD_WP_REG		(0X8E)  /*	控制寄存器的位 7是写保护位，前 7位（位 0至位 6）被强制为 0且读取时总是读 0在任
											何对时钟或 RAM的写操作以前，位 7必须为 0.当为高时，写保护位禁止任何寄存器的写操
											作.初始加电状态未定义.因此，在试图写器件之前应该清除 WP位.*/
#define DS1302_CMD_CLKBURST_REG	(0XBE)	//时钟脉冲串
#define DS1302_CMD_SRMBURST_REG	(0XFE)	//RAM脉冲串

#define DS1302_RAM_ENDADDR		0xFC	//RAM最后的地址
#define DS1302_RAM_BYTES		31		//RAM大小为31个字节

struct ds1302_clock {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t date;
	uint8_t month;
	uint8_t day;
	uint8_t year;
	uint8_t wp;
} __attribute__((packed));

/*
* timespec_t is non-portable, so this
* structure must be used instead
*/
typedef struct __attribute__((packed)) {
	unsigned int tv_sec;
	unsigned int tv_nsec;
} timestamp_t;

void bsp_InitDS1302(void);
int ds1302_clock_halt(void);
int ds1302_clock_resume(void);
uint8_t ds1302_get_seconds(void);
int ds1302_set_seconds(uint8_t seconds);
uint8_t ds1302_get_minutes(void);
int ds1302_set_minutes(uint8_t minutes);
int ds1302_set_12hr(void);
int ds1302_set_24hr(void);
uint8_t ds1302_get_date(void);
int ds1302_set_date(uint8_t date);
uint8_t ds1302_get_month(void);
int ds1302_set_month(uint8_t month);
uint8_t ds1302_get_day(void);
int ds1302_set_day(uint8_t day);
uint8_t ds1302_get_year(void);
int ds1302_set_year(uint8_t year);
int ds1302_write_ram(uint8_t address, uint8_t *data, int datalen);
int ds1302_read_ram(uint8_t address, uint8_t *data, int datalen);
int ds1302_wp_enable(void);
int time_to_ds1302_clock(time_t *time, struct ds1302_clock *clock);
int ds1302_clock_write_burst(struct ds1302_clock *clock);
int ds1302_clock_read_burst(struct ds1302_clock *clock);
int ds1302_clock_to_time(time_t *time, struct ds1302_clock *clock);
int ds1302_write_ram_burst(uint8_t* data);
int ds1302_read_ram_burst(uint8_t* data);

void clock_get_time(timestamp_t * time);
uint32_t clock_get_time_nopara(void);
void clock_set_time(timestamp_t * time);
void clock_get_monotonic(timestamp_t * time);
void rtc_time_get(timestamp_t * timestamp);
int timesync_nopara(void);
int timesync(time_t sec);
int obc_timesync(void);