//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/ins/uart/uart.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 14-01-19
#ifndef SRC_INS_UART_UART_H_
#define SRC_INS_UART_UART_H_

#include "xscugic.h"
#include "xuartps.h"

void uart_pre_init(u32 baud_rate);
void uart_init(XScuGic *intc_p, XUartPs_Handler isr, u32 baud_rate);
void uart_set_baudrate(u32 baud_rate);
void uart_enable_irq(XScuGic *intc_p);
u32  uart_tx(u8 *data, u32 num_bytes);
u32  uart_rx(u8 *rx_buf, u32 num_bytes);

#endif /* SRC_INS_UART_UART_H_ */
