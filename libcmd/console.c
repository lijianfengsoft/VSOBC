////////////////////////////////////////////////////////////////////////////////
//	功能： 调试终端功能模块
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#include "FreeRTOS.h"
#include "task.h"
#include "command.h"

#include "usart.h"
#include "queue.h"
#include "console.h"

#include "graduate.h"

extern struct USART1_TypeDefStruct Console_Usart;
extern xQueueHandle  console_queue;

#if	CPU_A
static char * hostname = "CPU-A";
#endif
#if CPU_B
static char * hostname = "CPU-B";
#endif
#if	CPU_C
static char * hostname = "obc";
#endif

//static char * hostname = "CPU-A";
static const char prompt[] = "# ";

static int escape = CONSOLE_NORMAL;

#ifdef CONSOLE_HISTORY_ENABLE
static int history_elements = 0;
static int history_cur = 0;
static int history_browse = 0;
static char * history[CONSOLE_HISTORY_ELEMENTS];
#endif

static int size = 0;
static int pos = 0;
static char buf[CONSOLE_BUFSIZ];

static uint8_t(*console_getc)(void) = usart_getc;
static void(*console_putc)(uint8_t) = usart_putc;

static const char backspace_char = '\b';
static const char space_char = ' ';
static const char cr_char = '\r';
static const char nl_char = '\n';



static void console_write(const char *str, int length) {
	int i;

	for (i = 0; i < length; i++)
		console_putc(str[i]);
}

static void console_prompt(void) {
	console_write(hostname, strlen(hostname));
	console_write(prompt, strlen(prompt));
}

void console_set_hostname(char *host) {
	hostname = host;
}

static void console_reset(void) {
	size = 0;
	pos = 0;
	buf[pos] = '\0';

	console_prompt();
}

static void console_rewind(void) {
	int plen = strlen(prompt) + strlen(hostname);
	console_write(&cr_char, 1);
	while (size-- + plen)
		console_write(&space_char, 1);
	pos = 0;
	size = 0;
	console_write(&cr_char, 1);
}

void console_clear(void) {
	console_rewind();
	console_reset();
}

static void console_update(void) {
	console_rewind();
	console_prompt();
	size = strlen(buf);
	pos = size;
	console_write(buf, size);
}

#ifdef CONSOLE_HISTORY_ENABLE

static void console_history_add(void) {

	if (history[history_cur] != NULL)
		vPortFree(history[history_cur]);

	history[history_cur] = strdup(buf);

	history_browse = 0;
	history_cur = (history_cur + 1) % CONSOLE_HISTORY_ELEMENTS;

	if (history_elements < CONSOLE_HISTORY_ELEMENTS)
		history_elements++;

}

static void console_last_line(void) {

	if (history_elements < 1)
		return;

	if (history_browse >= history_elements)
		return;

	console_rewind();
	history_browse++;
	strcpy(buf, history[(history_cur - history_browse + CONSOLE_HISTORY_ELEMENTS) % CONSOLE_HISTORY_ELEMENTS]);

	console_update();

}

static void console_next_line(void) {

	if (history_elements < 1)
		return;

	if (history_browse < 1)
		return;

	console_rewind();
	history_browse--;
	if (history_browse > 0)
		strcpy(buf, history[(history_cur - history_browse + CONSOLE_HISTORY_ELEMENTS) % CONSOLE_HISTORY_ELEMENTS]);
	else
		buf[0] = '\0';

	console_update();

}

#endif

static void console_forward_char(void) {
	if (pos < size) {
		console_write(&buf[pos], 1);
		pos++;
	}
}

static void console_end_of_line(void) {
	while (pos < size)
		console_forward_char();
}

static void console_backward_char(void) {
	if (pos > 0) {
		pos--;
		console_write(&backspace_char, 1);
	}
}

static void console_beginning_of_line(void) {
	while (pos)
		console_backward_char();
}

static void console_newline(void) {
	console_write(&cr_char, 1);
	console_write(&nl_char, 1);
}

static void console_execute(void) {
	console_newline();
	if (size > 0) {
		command_run(buf);
#ifdef CONSOLE_HISTORY_ENABLE
		console_history_add();
#endif
	}
	console_newline();//add by Wl
	console_reset();
}

static void console_complete(void) {
	/* We don't expand in the middle of a line */
	if (size != pos)
		return;

	int ret = command_complete(buf);
	if (ret == CMD_ERROR_AMBIGUOUS) {
		console_newline();
		command_help(buf);
	}
	else if (ret == CMD_ERROR_COMPLETE) {
		console_newline();
		command_usage(buf);
	}

	if (ret != CMD_ERROR_NOTFOUND)
		console_update();
}

static void console_insert(char c) {

	int i;
	int diff = size - pos;

	if (size + 1 >= CONSOLE_BUFSIZ)
		return;

	memmove(&buf[pos + 1], &buf[pos], diff);
	buf[pos] = c;

	console_write(&buf[pos], diff + 1);
	for (i = 0; i < diff; i++)
		console_write(&backspace_char, 1);

	size++;
	pos++;
	buf[size] = '\0';

}

static void console_insert_overwrite(char c) {

	buf[pos++] = c;

	if (pos > size)
		size++;

	console_write(&c, 1);

}

static void console_delete(void) {

	int i;
	int diff = size - pos;

	/* Nothing to delete */
	if (size == pos)
		return;

	size--;
	memmove(&buf[pos], &buf[pos + 1], diff - 1);
	buf[size] = '\0';

	console_write(&buf[pos], diff - 1);
	console_write(&space_char, 1);
	for (i = 0; i < diff; i++)
		console_write(&backspace_char, 1);

}

static void console_backspace(void) {
	if (pos < 1)
		return;

	console_backward_char();
	console_delete();
}

static void console_kill_line(void) {
	int i;
	int diff;

	diff = size - pos;

	if (diff == 0)
		return;

	for (i = 0; i < diff; i++)
		console_write(&space_char, 1);
	for (i = 0; i < diff; i++)
		console_write(&backspace_char, 1);

	memset(&buf[pos], 0, diff);
	size = pos;
}

static void console_kill_line_from_beginning(void) {
	console_beginning_of_line();
	console_kill_line();
}

/*static void console_forward_kill_word(void) {
while (pos != size && buf[pos] == ' ')
console_delete();
while (pos != size && buf[pos] != ' ')
console_delete();
}*/

static void console_backward_kill_word(void) {
	while (pos > 0 && buf[pos - 1] == ' ')
		console_backspace();
	while (pos > 0 && buf[pos - 1] != ' ')
		console_backspace();
}

static void console_transpose_chars(void) {
	char c1, c2;

	if (size < 2 || pos < 1)
		return;

	if (pos == size) {
		c1 = buf[pos - 1];
		c2 = buf[pos - 2];

		console_backward_char();
		console_backward_char();
		console_insert_overwrite(c1);
		console_insert_overwrite(c2);
	}
	else {
		c1 = buf[pos];
		c2 = buf[pos - 1];

		console_backward_char();
		console_insert_overwrite(c1);
		console_insert_overwrite(c2);
	}
}

static void console_suspend(void) {
	/* We cannot suspend */
}

static void console_interrupt(void) {
	/* We cannot interrupt */
}

void debug_console(void *pvParameters __attribute__((unused))) {

	char c;

	printf("\r\n\r\n");
	console_reset();

	while (1) {

		c = (char)console_getc();		
		switch (c) {
		case CONTROL('A'):
			console_beginning_of_line();
			break;
		case CONTROL('B'):
			console_backward_char();
			break;
		case CONTROL('C'):
			console_interrupt();
			break;
		case CONTROL('D'):
			console_delete();
			break;
		case CONTROL('E'):
			console_end_of_line();
			break;
		case CONTROL('F'):
			console_forward_char();
			break;
		case CONTROL('K'):
			console_kill_line();
			break;
		case CONTROL('L'):
			console_clear();
			break;

#ifdef CONSOLE_HISTORY_ENABLE
		case CONTROL('N'):
			console_next_line();
			break;
		case CONTROL('P'):
			console_last_line();
			break;
#endif
		case CONTROL('T'):
			console_transpose_chars();
			break;
		case CONTROL('U'):
			console_kill_line_from_beginning();
			break;
		case CONTROL('W'):
			console_backward_kill_word();
			break;
		case CONTROL('Z'):
			console_suspend();
			break;
		case CONTROL('H'):
		case 0x7f:
			console_backspace();
			break;
		case '\r':
		case '\n':
			console_execute();
			break;
		case '\t':
			console_complete();
			break;
		case '\033':
			escape = CONSOLE_ESCAPE;
			break;
		default:
			if (escape == CONSOLE_ESCAPE) {
				if ((c == '[') || (c == 'O')) {
					c = console_getc();
					if (c == 'F')
						console_end_of_line();
					if (c == 'H')
						console_beginning_of_line();
					if (c == 'A')
						console_last_line();
					if (c == 'B')
						console_next_line();
					if (c == 'C')
						console_forward_char();
					if (c == 'D')
						console_backward_char();
					if (c == '1')
						if (console_getc() == '~')
							console_beginning_of_line();
					if (c == '3')
						if (console_getc() == '~')
							console_delete();
				}
				escape = CONSOLE_NORMAL;
				break;
			}

			if (isprint((unsigned char)c))
				console_insert(c);
			break;
		}
	}
}

int console_exit(void) {

	return 0;
}

int console_init(void) {

	extern void cmd_dfl_setup(void);
	cmd_dfl_setup();

	return 0;
}

uint8_t usart_getc(void)
{
	uint8_t recbyte = 0;
	xQueueReceive(console_queue, &recbyte, portMAX_DELAY);

	return recbyte;
}

void usart_putc(uint8_t sendbyte)
{
	Usart_write(&Console_Usart, sendbyte);
}

int usart_messages_waiting(void) {
	return uxQueueMessagesWaiting(console_queue);
}




