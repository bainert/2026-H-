#include "uart.h"
#include "ti_msp_dl_config.h"

//发送字符
void uart_send_char(UART_Regs *uart, const uint8_t data)
{
    DL_UART_transmitDataBlocking(uart, data);
}

//发送字符串
void uart_send_string(UART_Regs *uart, const char *str)
{
    while (*str != '\0')
    {
        uart_send_char(uart, *str++);
    }
    while (DL_UART_isBusy(uart));
}

//串口接收中断处理函数
void UART_0_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
    case DL_UART_IIDX_RX:
    {
        //将接收的字符发送回发送端
        uint8_t data = DL_UART_receiveDataBlocking(UART_0_INST);
        uart_send_char(UART_0_INST, data);
        break;
    }
    
    default:
        break;
    }
}