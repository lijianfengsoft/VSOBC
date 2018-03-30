/***************************************************************************
*  功能：初始化
*  版本：V1.0
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/6 22:07 
*****************************************************************************/                                                    
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "bsp_init.h"
#include "usart.h"
#include "console.h"
#include "command.h"
#include "system.h"
#include "sdio_sdcard.h"
#include "ff.h"
#include "bsp_intemp.h"
#include "delay.h"
#include "bsp_cpuflsh.h"
#include "bsp_norflash.h"
#include "bsp_extsram.h"
#include "bsp_ds1302.h"
#include "bsp_pca9665.h"
#include "bsp_hklist.h"
#include "bsp_hk.h"

#include "time.h"

void bsp_init(void)
{
	FIL fil;
	FATFS  FatFs;
	FRESULT res;
	UINT bww;
	BYTE work[FF_MAX_SS];
	int temp;
	char buf[100];
	memset(buf, 0, 100);

	timestamp_t timestamp;
	struct ds1302_clock clock;
	time_t t = 0;


	Stm32_Clock_Init(336, 25, 2, 7);

	usart_init(115200);

	command_init();

	console_init();

	intemp_adcinit();

	delay_init(168);

	Norflash_Init();

	//EXT_SRAMInit();

	bsp_InitDS1302();
/*将DS1302时间初始化为1970年1月1号 00:00:00:00*/
	//time(&t);为什么会返回-1错误，可能的原因是与中国所在时区有关
	//time_to_ds1302_clock(&t, &clock);
	//ds1302_clock_write_burst(&clock);

	rtc_time_get(&timestamp);	//获取DS1302的时间并转换成为自1970年1月1日以来持续时间的秒数
	clock_set_time(&timestamp);	//设置本地时间偏移（此处时时钟节拍计数应为0）

	printf("RTC Time is: %s\r\n", ctime((time_t *)&timestamp.tv_sec));


	ad7490_spi_init();
	AD7490_Read();

	PCA9665_IO_Init();
	i2c_init(0, I2C_MASTER, 0x1A, 400 , 5, 5, NULL);
	pca9665_isr_init();
	 
	temp = (int)Get_Temprate();
	printf("\r\ntemprate=%d\r\n",temp);
	while (SD_Init());
	show_sdcard_info();
	/*挂载文件系统*/
	f_mount(&FatFs, "0:", 1);
	f_mkdir("0:hk");
	/*格式化SD卡，f_mkfs() 应在 f_mount()之后，否则出现，无工作区的错位。不经意容易犯错。*/
	//f_mkfs("0:",FM_ANY,0,work,sizeof work);

		if ((res = f_open(&fil, "0:/message.txt", FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
		{
			f_write(&fil, "alientek test lijianfeng taoye", 50, &bww);

			f_close(&fil);

			res = f_open(&fil, "0:/message.txt", FA_READ);

			f_read(&fil, buf, 100, &bww);
			f_close(&fil);

			printf("%s\r\n", buf);
		}	


		NOR_ReadID();

		hk_list_init(&hk_list);
		hk_list_recover();
		vTelemetryFileManage(&hk_list);
}
