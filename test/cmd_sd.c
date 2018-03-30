/***************************************************************************
*  功能：测试SD操作
*  版本：
*  迭代：
                                                 南京理工大学微纳卫星中心   
                                                    LJF：2018/1/25 20:18 
*****************************************************************************/    
#include "FreeRTOS.h"

#include "command.h"
#include "sdio_sdcard.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "ff.h"
#include "string.h"
#include "stdio.h"
#include <ctype.h>
#include "cmd.h"

int cmd_fat_write(struct command_context *ctx) 
{
	char * args = command_args(ctx);
	char * filename;
	char *data = (char*)pvPortMalloc(100);

	if (sscanf(args, "%s %s", filename,data) != 2)
		return CMD_ERROR_SYNTAX;

	FIL myfile;
	UINT byterwrite;
	char pre_path[40] = {0};

	strcat(pre_path, "/");
	strcat(pre_path, filename);

	printf("the filename is: %s\r\n", filename);

	int result = f_open(&myfile, pre_path, FA_WRITE | FA_OPEN_ALWAYS);
	if (result != FR_OK) 
	{
		printf("open file error ,result is :%u\r\n", result);
		return CMD_ERROR_FAIL;
	}

	result = f_write(&myfile, data, strlen(data), &byterwrite);

	f_close(&myfile);

	printf("written num: %u\r\n", byterwrite);
	if (result == FR_OK) 
	{
		printf("write %s to %s\r\n", data, filename);
	}
	vPortFree(data);
	data = NULL;
	return CMD_ERROR_NONE;
}


int cmd_fat_read(struct command_context *ctx) 
{

	FIL myfile;
	UINT byteread;
	char   buffer;
	char * args = command_args(ctx);
	char * filename;
	char pre_path[40] = {0};

	if (sscanf(args, "%s", filename) != 1)
		return CMD_ERROR_SYNTAX;

	strcat(pre_path, "/");
	strcat(pre_path, filename);

	printf("the filename is: %s\r\n", pre_path);

	int result = f_open(&myfile, pre_path, FA_READ | FA_OPEN_EXISTING);

	if (result != FR_OK) 
	{
		printf("the filename is not existing\r\n");
		printf("open file error ,result is :%u\r\n", result);
		return CMD_ERROR_FAIL;
	}

	int i = 0;

	for (;;)
	{
		result = f_read(&myfile, &buffer, 1, &byteread);
		if (result == FR_OK) 
		{
			if (byteread == 0) break;
			i++;
			if (isprint(buffer))
			{
				printf("%c", buffer);
			}
			else
			{
				printf("*");
			}
		}
		else
		{
			printf("read failed\r\n");
			printf("read error ,result is :%u\r\n", result);
			f_close(&myfile);

			return CMD_ERROR_FAIL;
		}
	}
	printf("\r\n");
	printf("read byte number is: %d\r\n", i);
	f_close(&myfile);

	return CMD_ERROR_NONE;

}


command_t __sub_command sd_subcommands[] = {
	{
		.name = "read",
		.help = "read a file",
		.usage = "<file name>",
		.handler = cmd_fat_read,
	},{
		.name = "write",
		.help = "read a file",
		.usage = "<file name><write string>",
		.handler = cmd_fat_write,
	}
};

command_t __root_command cmd_sd[] = {
	{
		.name = "sd",
		.help = "sd fatfs system",
		.chain = INIT_CHAIN(sd_subcommands),
	},
};

void cmd_sdcard_setup(void)
{
	command_register(cmd_sd);
}
