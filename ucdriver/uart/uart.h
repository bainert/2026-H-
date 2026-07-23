#ifndef __UART_H__
#define __UART_H__

#include "ti_msp_dl_config.h"

//发送字符
void uart_send_char(UART_Regs *uart, const uint8_t data);
//发送字符串
void uart_send_string(UART_Regs *uart, const char *str);

#endif
