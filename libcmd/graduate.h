#pragma once
////////////////////////////////////////////////////////////////////////////////
//	功能： 卫星平台通用配置定义头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_CUBESAT_H_
#define INCLUDE_CUBESAT_H_

#define CPU_A		0
#define CPU_B		0
#define CPU_C		1

#if	CPU_A
#define CONSOLE COM2//4		//调试终端
#endif
#if CPU_B
#define CONSOLE COM1		//调试终端
#endif
#if	CPU_C
#define CONSOLE COM1		//调试终端
#endif


#define CONSOLE_HISTORY_ENABLE 1
#define CONSOLE_HISTORY_ELEMENTS 10
#define CONFIG_DRIVER_DEBUG 1

int gpa_ts;

#endif


