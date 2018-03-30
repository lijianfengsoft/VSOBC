#pragma once
/*
* driver_debug.h
*
*  Created on: 03/03/2011
*      Author: johan
*/

#ifndef DRIVER_DEBUG_H_
#define DRIVER_DEBUG_H_

#include "command.h"

typedef enum driver_debug_e {
	DEBUG_I2C = 0,
	DEBUG_OBC = 1,
	DEBUG_SPI = 2,
	DEBUG_CAMERA = 3,
	DEBUG_ICD = 4,
	DEBUG_HK = 5,
	DEBUG_TTC = 6,
	DEBUG_FLASH = 7,
	DEBUG_ROUTER = 8,
	DEBUG_ENUM_MAX = DEBUG_ROUTER,
} driver_debug_t;

typedef enum driver_debug_value_e {
	DRIVER_DEBUG_OFF = 0,
	DRIVER_DEBUG_ON = 1,
} driver_debug_value_t;

#define driver_debug(driver, format, ...) { if (driver_debug_enabled(driver)) { printf(format, ##__VA_ARGS__);} };
unsigned int driver_debug_enabled(driver_debug_t driver);
void driver_debug_toggle(driver_debug_t driver);
void driver_debug_set(driver_debug_t driver, driver_debug_value_t);
int cmd_driver_debug_toggle(struct command_context * context);

#endif /* DRIVER_DEBUG_H_ */
