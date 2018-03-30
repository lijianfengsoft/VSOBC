/*
* cmd_dfl.c
*
*  Created on: Sep 17, 2012
*      Author: johan
*/
#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <string.h>

#include "bsp_reset.h"
#include "bsp_obc_argvs_store.h"

#include "usart.h"
#include "graduate.h"


#include "driver_debug.h"
#include "command.h"


static uint8_t(*console_getc)(void) = usart_getc;
static void(*console_putc)(uint8_t) = usart_putc;



int help_handler(struct command_context * context) {
	command_help(command_args(context));
	return CMD_ERROR_NONE;
}

int sleep_handler(struct command_context * context) {
	unsigned long sleep_ms;

	if (context->argc != 2)
		return CMD_ERROR_SYNTAX;

	sleep_ms = atoi(context->argv[1]);

	if (sleep_ms < 1)
		return CMD_ERROR_SYNTAX;

	vTaskDelay(sleep_ms * (configTICK_RATE_HZ / 1000.0));

	return CMD_ERROR_NONE;
}


int watch_handler(struct command_context * context) {

	int sleep_ms = atoi(context->argv[1]);

	if (sleep_ms < 1)
		return CMD_ERROR_SYNTAX;

	printf("Execution delay: %d\r\n", sleep_ms);

	char * new_command = strstr(command_args(context), " ");

	if (new_command == NULL)
		return CMD_ERROR_SYNTAX;
	else
		new_command = new_command + 1;

	printf("Command: %s\r\n", new_command);

	while (1) {

		if (usart_messages_waiting())
			break;

		command_run(new_command);

		vTaskDelay(sleep_ms * (configTICK_RATE_HZ / 1000.0));

	}

	return CMD_ERROR_NONE;

}

#define CONTROL(X)  ((X) - '@')

int batch_handler(struct command_context * ctx __attribute__((unused))) {

	char c;
	int quit = 0, execute = 0;
	unsigned int batch_size = 100;
	unsigned int batch_input = 0;
	unsigned int batch_count = 0;
	char * batch[20] = {};
	printf("Type each command followed by enter, hit ctrl+e to end typing, ctrl+x to cancel:\r\n");

	/* Wait for ^q to quit. */
	while (quit == 0) {

		/* Get character */
		c = console_getc();

		switch (c) {

			/* CTRL + X */
		case 0x18:
			quit = 1;
			break;

			/* CTRL + E */
		case 0x05:
			execute = 1;
			quit = 1;
			break;

			/* Backspace */
		case CONTROL('H'):
		case 0x7f:
			if (batch_input > 0) {
				console_putc('\b');
				console_putc(' ');
				console_putc('\b');
				batch_input--;
			}
			break;

		case '\r':
			console_putc('\r');
			console_putc('\n');
			if ((batch[batch_count] != NULL) && (batch_input < batch_size))
				batch[batch_count][batch_input++] = '\r';
			if ((batch[batch_count] != NULL) && (batch_input < batch_size))
				batch[batch_count][batch_input++] = '\0';
			batch_count++;
			batch_input = 0;
			if (batch_count == 20)
				quit = 1;
			break;

		default:
			console_putc(c);
			if (batch[batch_count] == NULL) {
				batch[batch_count] = calloc(1, 1);
			}

			if ((batch[batch_count] != NULL) && (batch_input < batch_size))
				batch[batch_count][batch_input++] = c;
			break;
		}
	}

	if (execute) {
		printf("\r\n");
		for (unsigned int i = 0; i <= batch_count; i++) {
			if (batch[i])
				printf("[%02u] %s\r\n", i, batch[i]);
		}
		printf("Press ctrl+e to execute, or any key to abort\r\n");
		c = console_getc();
		if (c != 0x05)
			execute = 0;
	}

	/* Run/Free batch job */
	for (unsigned int i = 0; i <= batch_count; i++) {
		if (execute && batch[i]) {
			printf("EXEC [%02u] %s\r\n", i, batch[i]);
			command_run(batch[i]);
		}
		free(batch[i]);
	}

	return CMD_ERROR_NONE;

}

int cpu_reset_handler(struct command_context * context __attribute__((unused))) {

	obc_save_t obc_save_test;

	STMFLASH_ReadByte(OBC_STORE_ADDR, (uint8_t*)&obc_save_test, sizeof(obc_save_test));

	printf("cpu_reset boot count=%d\r\n",obc_save_test.obc_boot_count);
	
	cpu_reset();

	return CMD_ERROR_NONE;
}

int ps_handler(struct command_context * context __attribute__((unused))) {

	signed char printbuffer[384];

	vTaskList((char *)printbuffer);

	printf("%s", printbuffer);

	return CMD_ERROR_NONE;
}

command_t __root_command cmd_dfl[] = {
	{
		.name = "help",
		.help = "Show help",
		.usage = "<command>",
		.handler = help_handler,
	},{
		.name = "sleep",
		.help = "Sleep X ms",
		.usage = "<time>",
		.handler = sleep_handler,
	},{
		.name = "watch",
		.help = "Run cmd at intervals, abort with key",
		.usage = "<n> <command>",
		.handler = watch_handler,
	},{
		.name = "batch",
		.help = "Run multiple commands",
		.handler = batch_handler,
	},
	{
		.name = "reset",
		.help = "Reset now",
		.handler = cpu_reset_handler,
	},{
		.name = "ps",
		.help = "List tasks",
		.handler = ps_handler,
	},{
#if CONFIG_DRIVER_DEBUG
		.name = "tdebug",
		.help = "Toggle driver debug",
		.usage = "<level>",
		.handler = cmd_driver_debug_toggle,
	}
#endif
};

void cmd_dfl_setup(void) {
	command_register(cmd_dfl);
}
