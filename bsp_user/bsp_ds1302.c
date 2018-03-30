/***************************************************************************
*  功能：配置DS1302驱动
*  版本：V 1.0
*  迭代：	SCLK：	PB15
			CE(RST):PB14----CE输入实现两个功能.第一, CE 开启允许对地址/命令序列的移位寄存器进行读写的控制逻辑.
							第二 CE 信号为单字节和多字节 CE数据传输提供了终止的方法.如果 CE输入为低电平,则所有
							数据传输终止,并且 I/O 口成高阻抗状态,在上电时, CE必须为逻辑 0直到 VCC 大于 2.0V
			I/O:	PG11----数据输入：输入写命令字的 8个 SCLK周期后 ，接下来的 8个 SCLK 周期的上升沿数据字节被输入，如
									 不慎发生， 多余的 SCLK 周期将被忽略，数据输入以位 0开始
							数据输出：输入读命令字的 8个 SCLK周期后, 随后的 8个 SCLK 周期的下降沿，一个数据字节被输出
命令字启动每一次数据传输. MSB (位 7)必须是逻辑 1. 如果是 0,
则禁止对 DS1302写入. 位 6 在逻辑 0时规定为时钟/日历数据,逻辑 1时为 RAM数据.
位 1 至 位 5 表示了输入输出的指定寄存器.LSB (位 0) 在逻辑0时为写操作(输出),逻辑
1时为读操作(输入).命令字以 LSB (位 0)开始总是输入.
时间和日历寄存器的内容是二进制编码的十进制（BCD）格式的(用4位二进制数来表示1位十进制数中的0~9这10个数码)
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/12 19:30 
*****************************************************************************/           
#include <time.h>


#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "delay.h"
#include "bsp_ds1302.h"

/* Local time offset */
static timestamp_t offset = { 0, 0 };

static int ds1302_write(uint8_t cmd, uint8_t *in, int inlen);
static inline uint8_t reverse_bits(uint8_t bits);
static int ds1302_io_read(void);

#define DS1302_BIT_DELAY_US		5

#define gpio_ce_high()		(GPIOB->BSRRL=GPIO_Pin_14)
#define gpio_ce_low()		(GPIOB->BSRRH=GPIO_Pin_14)

#define gpio_io_high()		(GPIOG->BSRRL=GPIO_Pin_11)
#define gpio_io_low()		(GPIOG->BSRRH=GPIO_Pin_11)

#define gpio_sclk_high()	(GPIOB->BSRRL=GPIO_Pin_15)
#define gpio_sclk_low()		(GPIOB->BSRRH=GPIO_Pin_15)

#define read_pin_io()		(GPIOG->IDR&GPIO_Pin_11)


void bsp_InitDS1302(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	uint8_t trickle = SET_TRICKLE_REGISTER;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOG, &GPIO_InitStruct);

	ds1302_write(DS1302_CMD_TRICKLE_REG, &trickle, sizeof(trickle));	//配置涓流寄存器

	ds1302_clock_resume();												//启动ds1302

	ds1302_wp_enable();													//启动时采用写保护

}

int ds1302_io_input(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	return 0;
}

int ds1302_io_output(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	return 0;
}

static void ds1302_enable(void) 
{
	gpio_ce_high();
}

static void ds1302_disable(void) 
{
	gpio_ce_low();
}

static void ds1302_io_high(void) 
{
	gpio_io_high();
}

static void ds1302_io_low(void) 
{
	gpio_io_low();
}

static void ds1302_sclk_high(void) {
	gpio_sclk_high();
}

static void ds1302_sclk_low(void) {
	gpio_sclk_low();
}

static void ds1302_io_write(int val)
{
	if (val)
		ds1302_io_high();
	else
		ds1302_io_low();
}


static void ds1302_write_byte(uint8_t byte) 
{
	int i;

	for (i = 0; i < 8; i++) {
		ds1302_io_write(byte & 0x01);
		delay_us(DS1302_BIT_DELAY_US);
		ds1302_sclk_high();
		delay_us(DS1302_BIT_DELAY_US);
		byte = byte >> 1;
		ds1302_sclk_low();
		delay_us(DS1302_BIT_DELAY_US);
	}
}

static int ds1302_write(uint8_t cmd, uint8_t *in, int inlen) 
{
	int i;

	ds1302_enable();

	/* Send command */
	ds1302_write_byte(cmd | DS1302_CMD_WRITE);

	/* Send data */
	for (i = 0; i < inlen; i++)
		ds1302_write_byte(in[i]);

	ds1302_disable();

	return 0;
}

static int ds1302_io_read(void) 
{
	return (read_pin_io() >> 11);
}

static uint8_t ds1302_read_byte(void) 
{
	int i;
	uint8_t byte = 0;

	for (i = 0; i < 8; i++) {
		byte = byte << 1;
		byte |= ds1302_io_read();
		delay_us(DS1302_BIT_DELAY_US);
		ds1302_sclk_high();
		delay_us(DS1302_BIT_DELAY_US);
		ds1302_sclk_low();
		delay_us(DS1302_BIT_DELAY_US);
	}

	return reverse_bits(byte);
}

static int ds1302_read(uint8_t cmd, uint8_t *out, int outlen)
{
	int i;

	ds1302_enable();

	/* Send command */
	ds1302_write_byte(cmd | DS1302_CMD_READ);

	/* Read data */
	ds1302_io_input();
	for (i = 0; i < outlen; i++)
		out[i] = ds1302_read_byte();
	ds1302_io_output();

	ds1302_disable();

	return 0;
}

int ds1302_clock_halt(void) 
{
	uint8_t seconds;
	ds1302_read(DS1302_CMD_SECONDS_REG, &seconds, sizeof(seconds));
	seconds |= DS1302_CH_MASK;
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_SECONDS_REG, &seconds, sizeof(seconds));
	ds1302_wp_enable();
	return 0;
}

int ds1302_clock_resume(void) 
{
	uint8_t seconds;
	ds1302_read(DS1302_CMD_SECONDS_REG, &seconds, sizeof(seconds));
	seconds &= ~DS1302_CH_MASK;
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_SECONDS_REG, &seconds, sizeof(seconds));
	ds1302_wp_enable();
	return 0;
}

int ds1302_wp_enable(void) 
{
	uint8_t cr = DS1302_WP_MASK;
	ds1302_write(DS1302_CMD_WP_REG, &cr, sizeof(cr));
	return 0;
}

int ds1302_wp_disable(void) 
{
	uint8_t cr = 0;
	ds1302_write(DS1302_CMD_WP_REG, &cr, sizeof(cr));
	return 0;
}

/* Convert from BCD values to something sane */
static inline uint8_t bcd_to_sane(uint8_t bcd) 
{
	return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static inline uint8_t sane_to_bcd(uint8_t sane)
{
	return ((sane / 10) << 4) + (sane % 10);
}

static inline uint8_t reverse_bits(uint8_t bits) 
{
	unsigned int b = bits;
	b = ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
	return (uint8_t)b;
}

uint8_t ds1302_get_seconds(void) 
{
	uint8_t seconds;

	ds1302_read(DS1302_CMD_SECONDS_REG, &seconds, sizeof(seconds));

	return bcd_to_sane(seconds & 0x7f);
}

int ds1302_set_seconds(uint8_t seconds) 
{
	uint8_t ch;

	if (seconds > 59)
		return -1;
	ds1302_read(DS1302_CMD_SECONDS_REG, &ch, sizeof(ch));		//主要是为了获取到CH的状态
	seconds = sane_to_bcd(seconds & 0x7f) | (ch & 0x80);		//保存前后CH状态与之前一致
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_SECONDS_REG, &seconds, sizeof(seconds));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_minutes(void)
{
	uint8_t minutes;

	ds1302_read(DS1302_CMD_MINUTES_REG, &minutes, sizeof(minutes));

	return bcd_to_sane(minutes);
}

int ds1302_set_minutes(uint8_t minutes) 
{
	if (minutes > 59)
		return -1;
	minutes = sane_to_bcd(minutes);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_MINUTES_REG, &minutes, sizeof(minutes));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_hour(void) 
{
	uint8_t hour;

	ds1302_read(DS1302_CMD_HOUR_REG, &hour, sizeof(hour));

	return bcd_to_sane(hour);
}

int ds1302_set_hour(uint8_t hour)
{
	if (hour > 23)
		return -1;
	hour = sane_to_bcd(hour);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_HOUR_REG, &hour, sizeof(hour));
	ds1302_wp_enable();
	return 0;
}

int ds1302_set_12hr(void) 
{
	uint8_t hours;

	ds1302_read(DS1302_CMD_HOUR_REG, &hours, sizeof(hours));
	hours |= DS1302_12HR_MASK;
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_HOUR_REG, &hours, sizeof(hours));
	ds1302_wp_enable();
	return 0;
}

int ds1302_set_24hr(void)
{
	uint8_t hours;

	ds1302_read(DS1302_CMD_HOUR_REG, &hours, sizeof(hours));
	hours &= ~DS1302_12HR_MASK;
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_HOUR_REG, &hours, sizeof(hours));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_date(void) 
{
	uint8_t date;

	ds1302_read(DS1302_CMD_DATE_REG, &date, sizeof(date));

	return bcd_to_sane(date);
}

int ds1302_set_date(uint8_t date)
{
	if (date < 1 || date > 31)
		return -1;
	date = sane_to_bcd(date);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_DATE_REG, &date, sizeof(date));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_month(void) 
{
	uint8_t month;

	ds1302_read(DS1302_CMD_MONTH_REG, &month, sizeof(month));

	return bcd_to_sane(month);
}

int ds1302_set_month(uint8_t month)
{
	if (month < 1 || month > 12)
		return -1;
	month = sane_to_bcd(month);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_MONTH_REG, &month, sizeof(month));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_day(void) 
{
	uint8_t day;

	ds1302_read(DS1302_CMD_DAY_REG, &day, sizeof(day));

	return bcd_to_sane(day);
}

int ds1302_set_day(uint8_t day)
{
	if (day < 1 || day > 7)
		return -1;
	day = sane_to_bcd(day);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_DAY_REG, &day, sizeof(day));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_year(void) 
{
	uint8_t year;

	ds1302_read(DS1302_CMD_YEAR_REG, &year, sizeof(year));

	return bcd_to_sane(year);
}

int ds1302_set_year(uint8_t year) 
{
	if (year > 99)
		return -1;
	year = sane_to_bcd(year);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_YEAR_REG, &year, sizeof(year));
	ds1302_wp_enable();
	return 0;
}

/**************************************************
 *  功能:以脉冲模式写时钟
 *  日期:2018/1/12 22:51
 *  输入:时钟结构体，顺序是确定的
 *  输出:
 *  注意点:脉冲模式必须是8个时钟寄存器一起操作，从0-7，RAM无需写满31字节
 *************************************************/
int ds1302_clock_write_burst(struct ds1302_clock *clock)
{
	uint8_t old_seconds;

	if (!clock)
		return -1;

	/* Read minutes to get CH value */
	old_seconds = ds1302_get_seconds();

	clock->seconds = sane_to_bcd(clock->seconds & 0x7f) | (old_seconds & 0x80);
	clock->minutes = sane_to_bcd(clock->minutes);
	clock->hour = sane_to_bcd(clock->hour);
	clock->date = sane_to_bcd(clock->date);
	clock->month = sane_to_bcd(clock->month);
	clock->day = sane_to_bcd(clock->day);
	clock->year = sane_to_bcd(clock->year);
	clock->wp = 0;

	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLKBURST_REG, (uint8_t *)clock, sizeof(*clock));
	ds1302_wp_disable();
	return 0;
}

/**************************************************
 *  功能:以脉冲串形式读取时钟
 *  日期:2018/1/12 22:55
 *  输入:读取时钟数据存放的结构体
 *  输出:
 *  注意点:
 *************************************************/
int ds1302_clock_read_burst(struct ds1302_clock *clock) {
	if (!clock)
		return -1;

	ds1302_read(DS1302_CMD_CLKBURST_REG, (uint8_t *)clock, sizeof(*clock));

	clock->seconds = bcd_to_sane(clock->seconds & 0x7f);
	clock->minutes = bcd_to_sane(clock->minutes);
	clock->hour = bcd_to_sane(clock->hour);
	clock->date = bcd_to_sane(clock->date);
	clock->month = bcd_to_sane(clock->month);
	clock->day = bcd_to_sane(clock->day);
	clock->year = bcd_to_sane(clock->year);
	clock->wp = 0;

	return 0;
}


/**************************************************
 *  功能:写入ram
 *  日期:2018/1/12 23:07
 *  输入:address：待写入首地址大小为0-31	data:待写入收据首地址		datalen：待写入收据长度
 *  输出:
 *  注意点:开始为C0至FC（COH C2H C4H……FCH）故要从C0开始
 *************************************************/
int ds1302_write_ram(uint8_t address, uint8_t *data, int datalen) 
{
	int i;

	address = 2 * address + 0xC0;

	if (address + datalen > DS1302_RAM_ENDADDR)
		return -1;
	ds1302_wp_disable();
	for (i = 0; i < datalen; i++)
		ds1302_write(address + 2 * i, data + i, sizeof(data[i]));	//因为有读写，所以一个地址是隔个2
	ds1302_wp_enable();

	return 0;
}

/**************************************************
 *  功能:读取ram
 *  日期:2018/1/12 23:27
 *  输入:address：待读首地址	data：读出数据存储地址	datalen：要读取数据的长度
 *  输出:
 *  注意点:ram的地址需注意，同写
 *************************************************/
int ds1302_read_ram(uint8_t address, uint8_t *data, int datalen) 
{
	int i;

	address = 2 * address + 0xC0;

	if (address + datalen > DS1302_RAM_ENDADDR)
		return -1;

	for (i = 0; i < datalen; i++)
		ds1302_read(address + 2 * i, data + i, sizeof(data[i]));

	return 0;
}

/**************************************************
 *  功能:以脉冲串模式读ram
 *  日期:2018/1/12 23:25
 *  输入:读出的数据存放的首地址
 *  输出:
 *  注意点:一共读出31个字节
 *************************************************/
int ds1302_read_ram_burst(uint8_t* data) 
{
	ds1302_read(DS1302_CMD_SRMBURST_REG, data, DS1302_RAM_BYTES);

	return 0;
}

/**************************************************
 *  功能:以脉冲串模式写ram
 *  日期:2018/1/12 23:26
 *  输入:待写入字节首地址
 *  输出:
 *  注意点:无需一定为写31个字节，小于31个字节都可以
 *************************************************/
int ds1302_write_ram_burst(uint8_t* data) 
{
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_SRMBURST_REG, data, DS1302_RAM_BYTES);
	ds1302_wp_enable();
	return 0;
}


/**************************************************
 *  功能:获取系统时间并给clock结构体赋值
 *  日期:2018/1/12 23:29
 *  输入:time：系统时间结构体		clock：DS1302时间结构体
 *  输出:错误码 0成功
 *  注意点:
 *************************************************/
int time_to_ds1302_clock(time_t *time, struct ds1302_clock *clock)
{
	struct tm *t;

	t = gmtime(time);		//将参数time所指的time_t 结构中的信息转换成真实世界所使用的时间日期表示方法，然后将结果由结构tm 返回。


	if (t && clock)
	{
		clock->seconds = t->tm_sec;
		clock->minutes = t->tm_min;
		clock->hour = t->tm_hour;
		clock->date = t->tm_mday;
		clock->month = t->tm_mon + 1;			//？？？还不清楚
		clock->day = t->tm_wday;
		clock->year = t->tm_year;
		return 0;
	}
	else 
	{
		return -1;
	}
}

/**************************************************
*  功能:获取clock并给系统时间结构体赋值
*  日期:2018/1/12 23:29
*  输入:time：系统时间结构体	 clock：DS1302时间结构体
*  输出:错误码 0成功
*  注意点:
*************************************************/
int ds1302_clock_to_time(time_t *time, struct ds1302_clock *clock)
{
	struct tm t;

	if (!time || !clock)
		return -1;

	t.tm_sec = clock->seconds;
	t.tm_min = clock->minutes;
	t.tm_hour = clock->hour;
	t.tm_mday = clock->date;
	t.tm_mon = clock->month - 1;
	t.tm_year = clock->year;
	t.tm_isdst = 0;

	*time = mktime(&t);				//将参数t所指向的tm结构体数据转换成从公元1970年1月1日0时0分0秒算起至今的UTC时间所经历的秒数，返回秒数

	return 0;
}


/**************************************************
 *  功能:获取自vTaskStartScheduler以来的滴答计数被调用的总数
 *  日期:2018/1/12 23:43
 *  输入:timestamp_t结构体指针
 *  输出:
 *  注意点:
 *************************************************/
void clock_get_time(timestamp_t * time) 
{
	TickType_t clocks = xTaskGetTickCount();					//获取自vTaskStartScheduler以来的滴答计数被调用的总数

	time->tv_sec = offset.tv_sec + clocks / configTICK_RATE_HZ;
	time->tv_nsec = (offset.tv_nsec + (clocks % configTICK_RATE_HZ) * 1000000) % 1000000000;
}

/**************************************************
 *  功能:此函数与上一个函数功能相同，只是通过返回值获取，上函数以实参获取
 *  日期:2018/1/12 23:44
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
uint32_t clock_get_time_nopara(void) 
{

	timestamp_t  time;
	TickType_t clocks = xTaskGetTickCount();

	time.tv_sec = offset.tv_sec + clocks / configTICK_RATE_HZ;
	time.tv_nsec = (offset.tv_nsec + (clocks % configTICK_RATE_HZ) * 1000000) % 1000000000;

	return time.tv_sec;
}

void clock_set_time(timestamp_t * time)
{

	TickType_t clocks = xTaskGetTickCount();

	offset.tv_sec = time->tv_sec - clocks / configTICK_RATE_HZ;
	offset.tv_nsec = (time->tv_nsec - (clocks % configTICK_RATE_HZ) * 1000000) % 1000000000;
}

void clock_get_monotonic(timestamp_t * time)
{

	TickType_t clocks = xTaskGetTickCount();

	time->tv_sec = clocks / configTICK_RATE_HZ;
	time->tv_nsec = (clocks % configTICK_RATE_HZ) * (1000000000 / configTICK_RATE_HZ);
}


/**************************************************
 *  功能:获取此时DS1302_CLOCK的时刻然后赋值给系统时钟
 *  日期:2018/1/12 23:53
 *  输入:
 *  输出:
 *  注意点:公元1970年1月1日0时0分0秒算起至今的DS1302 RTC时间所经历的秒数
 *************************************************/
void rtc_time_get(timestamp_t * timestamp) {
	struct ds1302_clock clock;

	/* Get time from RTC */
	ds1302_clock_read_burst(&clock);
	ds1302_clock_to_time((time_t *)&timestamp->tv_sec, &clock);
	timestamp->tv_nsec = 0;
}


/**************************************************
 *  功能:task内部调用
 *  日期:2018/1/12 23:56
 *  输入:
 *  输出:
 *  注意点:
 *************************************************/
int timesync_nopara(void) {
	int result = 0;
	timestamp_t timestamp;
	TickType_t tick_count = xTaskGetTickCount();   //获取系统时钟节拍计数器的值

	/* Get time from RTC */
	rtc_time_get(&timestamp);

	offset.tv_sec = timestamp.tv_sec - tick_count / configTICK_RATE_HZ;
	offset.tv_nsec = timestamp.tv_nsec;

	//result = adcstimesync(timestamp.tv_sec);姿控时间同步，后面补上

	return result;
}

int timesync(time_t sec) 
{
	int result = 0;

	timestamp_t  time;
	struct ds1302_clock clock;

	time.tv_sec = sec;
	time.tv_nsec = 0;

	/* Sync */
	clock_set_time(&time);	

	time_to_ds1302_clock(&sec, &clock);
	ds1302_clock_write_burst(&clock);

	//result = adcstimesync(time.tv_sec);

	return result;
}

int obc_timesync(void) 
{
	timestamp_t timestamp;
	TickType_t tick_count = xTaskGetTickCount();

	/* Get time from RTC */
	rtc_time_get(&timestamp);

	offset.tv_sec = timestamp.tv_sec - tick_count / configTICK_RATE_HZ;
	offset.tv_nsec = timestamp.tv_nsec;

	return 0;
}