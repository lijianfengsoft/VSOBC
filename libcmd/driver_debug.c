/*
* driver_debug.c
*
*  Created on: 03/03/2011
*      Author: johan
*/

#include "graduate.h"
#include "driver_debug.h"

static unsigned char driver_debug_switch[DEBUG_ENUM_MAX + 1] = { 0 };

inline void driver_debug_toggle(driver_debug_t driver) {
	driver_debug_switch[driver] = (driver_debug_switch[driver] + 1) % 2;
}

inline void driver_debug_set(driver_debug_t driver, driver_debug_value_t value) {
	driver_debug_switch[driver] = value % 2;
}

inline unsigned int driver_debug_enabled(driver_debug_t driver) {
	return driver_debug_switch[driver];
}

int cmd_driver_debug_toggle(struct command_context * ctx) {
	char * args = command_args(ctx);
	unsigned int driver;
	if (sscanf(args, "%u", &driver) != 1)
		return CMD_ERROR_SYNTAX;

	if (driver > DEBUG_ENUM_MAX)
		return CMD_ERROR_FAIL;
	driver_debug_toggle(driver);
	printf("Debug %u = %u\r\n", driver, driver_debug_switch[driver]);
	return CMD_ERROR_NONE;
}

