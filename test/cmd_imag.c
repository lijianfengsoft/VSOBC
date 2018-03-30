
#include "command.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "ff.h"
#include "bsp_usart.h"
#include "usart.h"
#include "stdio.h"

int cmd_imag_tosd(struct command_context *ctx)
{
	FIL fil2;
	UINT bww;
	u8 i;
	static u8 flag = 10;

	f_mkdir("0:imag");
	char path[20];
	sprintf(path, "0:imag/Image%u", flag++);

	if (f_open(&fil2, path, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
		{
			if (f_write(&fil2, Picturedata, 500 * 1024, &bww) != 0)
				return -1;
		}
	f_close(&fil2);
	printf("bww=%d\r\n", bww);
	return 0;
}

command_t __sub_command imag_subcommands[] = {
	{
		.name = "tosd",
		.help = "storage a image to sd card",
		.usage = "<file name>",
		.handler = cmd_imag_tosd,
	}
};

command_t __root_command cmd_imag[] = {
	{
		.name = "imag",
		.help = "imag system",
		.chain = INIT_CHAIN(imag_subcommands),
	},
};

void cmd_imag_setup(void)
{
	command_register(cmd_imag);
}