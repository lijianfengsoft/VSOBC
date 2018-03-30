#pragma once
////////////////////////////////////////////////////////////////////////////////
//	���ܣ� command����ͷ�ļ�
//
//	�汾��V1.0
//  ������
//												�Ͼ�����ѧ΢����������
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "graduate.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define CMD_MAX_LEN_CMD		20
#define CMD_MAX_LEN_HELP	50
#define CMD_MAX_LEN_USAGE	50

#define CONSOLE_BUFSIZ 100

#define MAX_ARGC		25
#define MAX_TOKEN_SIZE 	50

#define CMD_ERROR_NONE 		0
#define CMD_ERROR_FAIL		(-1)
#define CMD_ERROR_SYNTAX	(-2)
#define CMD_ERROR_NOMEM		(-3)
#define CMD_ERROR_NOTFOUND 	(-100)
#define CMD_ERROR_AMBIGUOUS	(-101)
#define CMD_ERROR_CHAINED	(-102)
#define CMD_ERROR_EXTENDED	(-103)
#define CMD_ERROR_COMPLETE	(-104)

#define CMD_ESCAPE_NONE		0
#define CMD_ESCAPE_QUOTE	1
#define CMD_ESCAPE_DQUOTE	2

/* Command should not be available in the console */
#define CMD_NO_CONSOLE		0x00000001
#define CMD_HIDDEN			0x00000002

/* Command expects binary arguments */
#define CMD_BINARY_ARG		0x00010000

/* Memory sections */
#define __root_command __attribute__ ((section(".commands")))
#ifdef __AVR__
#define __sub_command PROGMEM
#else
#define __sub_command
#endif

#define INIT_CHAIN(__list) {.list = __list, .count = sizeof(__list)/sizeof(__list[0])}
#define command_register(__cmd) command_enable(__cmd, sizeof(__cmd)/sizeof(__cmd[0]))

#if defined(CONFIG_GOSH_CONST)
#define GOSH_CMD_STRUCT	const struct command
#else
#define GOSH_CMD_STRUCT	struct command
#endif

#define INIT_FIELDS(_fields) {.field = _fields, .fields = sizeof(_fields)/sizeof(_fields[0])}

#define ARG_STRING(_size, _dfl)  {.type = __ARG_STRING, .size = (_size * sizeof(char)) + 1, .dflt.str = _dfl}
#define ARG_BOOL(_dfl)	  {.type = __ARG_BOOL,   		.size = sizeof(uint8_t), 			.dflt.bl = _dfl}
#define ARG_UINT8(_dfl)	  {.type = __ARG_UINT8,			.size = sizeof(uint8_t), 			.dflt.u8 = _dfl}
#define ARG_INT8(_dfl)	  {.type = __ARG_INT8,			.size = sizeof(int8_t), 			.dflt.i8 = _dfl}
#define ARG_UINT16(_dfl)  {.type = __ARG_UINT16,		.size = sizeof(uint16_t), 			.dflt.u16 = _dfl}
#define ARG_INT16(_dfl)	  {.type = __ARG_INT16,			.size = sizeof(int16_t), 			.dflt.i16 = _dfl}
#define ARG_UINT32(_dfl)  {.type = __ARG_UINT32,		.size = sizeof(uint32_t), 			.dflt.u32 = _dfl}
#define ARG_INT32(_dfl)	  {.type = __ARG_INT32,			.size = sizeof(int32_t), 			.dflt.i32 = _dfl}
#define ARG_UINT64(_dfl)  {.type = __ARG_UINT64,		.size = sizeof(uint64_t), 			.dflt.u64 = _dfl}
#define ARG_INT64(_dfl)	  {.type = __ARG_INT64,			.size = sizeof(int64_t), 			.dflt.i64 = _dfl}
#define ARG_FLOAT(_dfl)	  {.type = __ARG_FLOAT,			.size = sizeof(float), 				.dflt.fl = _dfl}
#define ARG_DOUBLE(_dfl)  {.type = __ARG_DOUBLE,		.size = sizeof(double), 			.dflt.dl = _dfl}

typedef enum {
	__ARG_BOOL,
	__ARG_STRING,
	__ARG_UINT8,
	__ARG_INT8,
	__ARG_UINT16,
	__ARG_INT16,
	__ARG_UINT32,
	__ARG_INT32,
	__ARG_UINT64,
	__ARG_INT64,
	__ARG_FLOAT,
	__ARG_DOUBLE,
} field_t;

union field_default {
	const bool bl;
	const char *const str;
	const uint8_t u8;
	const int8_t i8;
	const uint16_t u16;
	const int16_t i16;
	const uint32_t u32;
	const int32_t i32;
	const uint64_t u64;
	const int64_t i64;
	const float fl;
	const double dl;
};

struct field_type {
	const field_t type;
	const uint8_t count;
	const unsigned int size;
	const union field_default dflt;
};

struct field {
	const char *const name;
	const struct field_type type;
	const char *const help;
};

struct message {
	struct field *field;
	unsigned int fields;
};

struct command_context {
	char **argv;
	int argc;
	void *arg;
	size_t arg_size;
	void *rpc_buf;
	GOSH_CMD_STRUCT *command;
};

typedef int(*command_handler_t)(struct command_context * context);

struct chain {
	GOSH_CMD_STRUCT *list;
	unsigned int count;
};

typedef GOSH_CMD_STRUCT{
#ifdef __AVR__
	char name[CMD_MAX_LEN_CMD];
char help[CMD_MAX_LEN_HELP];
char usage[CMD_MAX_LEN_USAGE];
#else
	const char * name;
const char * help;
const char * usage;
#endif
command_handler_t handler;
struct chain chain;
unsigned int mode;
#ifdef ENABLE_RPCX
struct message input;
struct message output;
#endif
uint32_t hash;
} command_t;

int command_build_argv(char *line, int *argc, char **argv);
char * command_args(struct command_context *ctx);
command_t * command_search(char *line);
int command_complete(char *line);
int command_run(char *line);
int command_help(char *line);
int command_usage(char *line);
int command_enable(command_t *cmd, int cmd_count);
int command_init(void);
void cmd_dfl_setup(void);


uint8_t usart_getc(void);
void usart_putc(uint8_t sendbyte);
int usart_messages_waiting(void);
#endif /* _COMMAND_H_ */
