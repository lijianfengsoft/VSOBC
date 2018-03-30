#pragma once

////////////////////////////////////////////////////////////////////////////////
//	功能： 调试终端功能模块头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////


#include <string.h>
#include <stdlib.h>
#include <ctype.h>


#define CONSOLE_LENGTH 0x2000		//队列块数

#define CONTROL(X)  ((X) - '@')		//@值为64，A位65

#define CONSOLE_NORMAL		0
#define CONSOLE_ESCAPE		1
#define CONSOLE_PRE_ESCAPE	2


void debug_console(void *pvParameters __attribute__((unused)));






