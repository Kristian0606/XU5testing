//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/timer/axi_timer.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 04-06-19
#include "xtmrctr.h"
#include "xparameters.h"
#include "axi_timer.h"

#define TIMER0_ID   XPAR_TMRCTR_0_DEVICE_ID
#define IRQ0_ID     XPAR_FABRIC_TMRCTR_0_VEC_ID
#define TIMER0_FREQ XPAR_TMRCTR_0_CLOCK_FREQ_HZ

#define TIMER1_ID   XPAR_TMRCTR_1_DEVICE_ID
#define IRQ1_ID     XPAR_FABRIC_TMRCTR_1_VEC_ID
#define TIMER1_FREQ XPAR_TMRCTR_1_CLOCK_FREQ_HZ

static XTmrCtr timer0_i;
static XTmrCtr timer1_i;

void axi_timer_init(void * isr0, void * isr1, XScuGic * intc_p) {
    const u8 RISING_EDGE = 0b11;
    const u8 PRIORITY    = 0xA0;

    /* AXI Timer 0 - Non-Periodic */
    XTmrCtr_Initialize(&timer0_i, TIMER0_ID);
    XTmrCtr_SetHandler(&timer0_i, isr0, (void *)(&timer0_i));
    XTmrCtr_SetOptions(&timer0_i, 0, XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION);
    XTmrCtr_SetOptions(&timer0_i, 1, XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION);

    XScuGic_SetPriorityTriggerType(intc_p, IRQ0_ID, PRIORITY, RISING_EDGE);
    XScuGic_Connect(intc_p, IRQ0_ID, XTmrCtr_InterruptHandler, &timer0_i);
    XScuGic_Enable(intc_p, IRQ0_ID);

    /* AXI Timer 1 - Periodic */
    XTmrCtr_Initialize(&timer1_i, TIMER1_ID);
    XTmrCtr_SetHandler(&timer1_i, isr1, (void *)(&timer1_i));
    XTmrCtr_SetOptions(&timer1_i, 0, XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION | XTC_AUTO_RELOAD_OPTION);
    XTmrCtr_SetOptions(&timer1_i, 1, XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION | XTC_AUTO_RELOAD_OPTION);

    XScuGic_SetPriorityTriggerType(intc_p, IRQ1_ID, PRIORITY, RISING_EDGE);
    XScuGic_Connect(intc_p, IRQ1_ID, XTmrCtr_InterruptHandler, &timer1_i);
    XScuGic_Enable(intc_p, IRQ1_ID);
}

static inline XTmrCtr * timer_ptr(axi_timer_t timer) {
    switch (timer) {
    default:
    case TIMER_SAFETY_ABPI:
    case TIMER_SAFETY_API:
        return &timer0_i;
    case TIMER_INV:
    case TIMER_INV_UNUSED:
        return &timer1_i;
    }
}

static inline u32 timer_freq(axi_timer_t timer) {
    switch (timer) {
    default:
    case TIMER_SAFETY_ABPI:
    case TIMER_SAFETY_API:
        return TIMER0_FREQ;
    case TIMER_INV:
    case TIMER_INV_UNUSED:
        return TIMER1_FREQ;
    }
}

void axi_timer_start(axi_timer_t timer, u32 msec) {
    XTmrCtr * ptr      = timer_ptr(timer);
    u8        idx      = axi_timer_idx(timer);
    u32       load_val = msec * (timer_freq(timer) / 1000U);

    XTmrCtr_Reset(ptr, idx);
    XTmrCtr_SetResetValue(ptr, idx, load_val);
    XTmrCtr_Start(ptr, idx);
}

void axi_timer_stop(axi_timer_t timer) {
    XTmrCtr * ptr = timer_ptr(timer);
    u8        idx = axi_timer_idx(timer);

    XTmrCtr_Stop(ptr, idx);
}
