//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/intc.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 14-01-19
#include "xil_exception.h"
#include "xparameters.h"
#include "intc.h"
#include "xipipsu.h"

#define INTC_ID XPAR_PSU_ACPU_GIC_DEVICE_ID

int intc_init(XScuGic * intc_p) {
    XScuGic_Config * cfg_p;

    u32 status = 0;

    cfg_p = XScuGic_LookupConfig(INTC_ID);
    if(cfg_p == NULL) {
        return XST_FAILURE;
    }
    
    status = XScuGic_CfgInitialize(intc_p, cfg_p, cfg_p->CpuBaseAddress);
    if(status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, intc_p);
    //Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_FIQ_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, intc_p);
    return XST_SUCCESS;
}

void intc_enable() {
    Xil_ExceptionEnable();
}

int intc_connect_isr(XScuGic * intc_p, u32 irq_id, void * isr) {
    u32 status = 0;
    status = XScuGic_Connect(intc_p, irq_id, (Xil_ExceptionHandler)isr, (void *)irq_id);
    if(status != XST_SUCCESS) {
        return XST_FAILURE;
    }


    XScuGic_SetPriorityTriggerType(intc_p, irq_id, 0xA0, 0x03); /* Figure out these magic numbers */
    XScuGic_Enable(intc_p, irq_id);
    return XST_SUCCESS;
}

int intc_connect_isr_to_ipi(XScuGic * intc_p, u32 irq_id, void * isr, XIpiPsu *ipi_p) {
    u32 status = 0;

    XScuGic_Connect(intc_p, irq_id, (Xil_ExceptionHandler)isr, (void *) ipi_p);
    if(status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XScuGic_SetPriorityTriggerType(intc_p, irq_id, 0xA0, 0x03); /* Figure out these magic numbers */
    XScuGic_Enable(intc_p, irq_id);

    return XST_SUCCESS;
}
