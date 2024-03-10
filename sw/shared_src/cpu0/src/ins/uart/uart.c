//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/ins/uart/uart.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 13-01-19
#include "xil_exception.h"
#include   "xparameters.h"
#include          "uart.h"

static XUartPs uart_i;
#define UART_0_ID   XPAR_XUARTPS_0_DEVICE_ID
#define UART_0_IRQ  XPAR_XUARTPS_0_INTR
#define T_TIMEOUT   10U

void uart_pre_init(u32 baud_rate){
    XUartPs_ResetHw(uart_i.Config.BaseAddress);

    XUartPs_Config *uart_cfg_p;
    uart_cfg_p = XUartPs_LookupConfig(UART_0_ID);

    XUartPs_CfgInitialize(&uart_i, uart_cfg_p, uart_cfg_p->BaseAddress);
    XUartPs_SetBaudRate(&uart_i, baud_rate);

    XUartPs_SetOperMode(&uart_i, XUARTPS_OPER_MODE_NORMAL);
    while(XUartPs_GetOperMode(&uart_i) != XUARTPS_OPER_MODE_NORMAL);
}


/**
 * Initializes an UART controller.
 *
 * @param[in/out] uart_p Pointer to the UART controller instance.
 *
 * @param [in/out] intc_p Pointer to the interrupt controller instance.
 *
 * @note Based on xuartps_intr_example.c
 */
void uart_init(XScuGic *intc_p,  XUartPs_Handler isr, u32 baud_rate){
    XUartPs_ResetHw(uart_i.Config.BaseAddress);
    XUartPs_Config *uart_cfg_p;

    uart_cfg_p = XUartPs_LookupConfig(UART_0_ID);
    XUartPs_CfgInitialize(&uart_i, uart_cfg_p, uart_cfg_p->BaseAddress);
    XUartPs_SetBaudRate(&uart_i, baud_rate);
    XUartPs_SetRecvTimeout(&uart_i, T_TIMEOUT);

    XScuGic_Connect(intc_p, UART_0_IRQ, (Xil_ExceptionHandler) XUartPs_InterruptHandler, &uart_i);
    XUartPs_SetHandler(&uart_i, isr, &uart_i);
    u32 event_mask = XUARTPS_IXR_TOUT | XUARTPS_IXR_RXOVR;
    XUartPs_SetFifoThreshold(&uart_i, 63);
    XUartPs_SetInterruptMask(&uart_i, event_mask);

    XUartPs_SetOperMode(&uart_i, XUARTPS_OPER_MODE_NORMAL);
    while(XUartPs_GetOperMode(&uart_i) != XUARTPS_OPER_MODE_NORMAL);
}

void uart_set_baudrate(u32 baud_rate) {
    XUartPs_SetBaudRate(&uart_i, baud_rate);
}

void uart_enable_irq(XScuGic *intc_p){
    XScuGic_Enable(intc_p, UART_0_IRQ);
}

u32 uart_tx(u8 *data, u32 num_bytes){
    u32 sent = XUartPs_Send(&uart_i, data, num_bytes);
    //while(!XUartPs_IsTransmitEmpty(&uart_i));
    return sent;
}

u32 uart_rx(u8 *rx_buf, u32 num_bytes){
    return XUartPs_Recv(&uart_i, rx_buf, num_bytes);
}
