/*
 * ttc_timer.c
 *
 *  Created on: 28 Feb 2022
 *      Author: DANKLORD420
 */

#include "ttc_timer.h"
#include "xttcps.h"

static XTtcPs epos_timer;
static XTtcPs initial_checkup_sequence_timer;

void ttc_timer_init(void) {
    XTtcPs_Config * epos_timer_config = XTtcPs_LookupConfig(XPAR_XTTCPS_0_DEVICE_ID);
    XTtcPs_CfgInitialize(&epos_timer, epos_timer_config, epos_timer_config->BaseAddress);
    XTtcPs_SetOptions(&epos_timer, XTTCPS_OPTION_INTERVAL_MODE | XTTCPS_OPTION_WAVE_DISABLE);
    XTtcPs_EnableInterrupts(&epos_timer, XTTCPS_IXR_INTERVAL_MASK);

    XTtcPs_Config * initial_checkup_sequence_timer_config = XTtcPs_LookupConfig(XPAR_XTTCPS_1_DEVICE_ID);
    XTtcPs_CfgInitialize(&initial_checkup_sequence_timer, initial_checkup_sequence_timer_config, initial_checkup_sequence_timer_config->BaseAddress);
    XTtcPs_SetOptions(&initial_checkup_sequence_timer, XTTCPS_OPTION_INTERVAL_MODE | XTTCPS_OPTION_WAVE_DISABLE);
    XTtcPs_EnableInterrupts(&initial_checkup_sequence_timer, XTTCPS_IXR_INTERVAL_MASK);
}

// Warning: Cannot be used with us > 1000000
void ttc_timer_schedule_interrupt(int us, const u8 timer_id) {
    if (us <= 0) {
        return;
    }
    XTtcPs * timer = NULL;
    if (TTC_TIMER_EPOS == timer_id) {
        timer = &epos_timer;
    } else if (TTC_TIMER_INITIAL_CHECKUP_SEQUENCE == timer_id) {
        timer = &initial_checkup_sequence_timer;
    } else {
        return;
    }

    XTtcPs_Stop(timer);
    u32       frequency = 1000000 / us;
    XInterval interval;
    u8        prescaler = 0xFF;
    XTtcPs_CalcIntervalFromFreq(timer, frequency, &interval, &prescaler);
    XTtcPs_SetInterval(timer, interval);
    XTtcPs_SetPrescaler(timer, prescaler);
    u32 interrupt_status = XTtcPs_GetInterruptStatus(timer);
    XTtcPs_ClearInterruptStatus(timer, interrupt_status);
    XTtcPs_Start(timer);
}

void ttc_timer_stop(const u8 timer_id) {
    XTtcPs * timer = NULL;
    if (TTC_TIMER_EPOS == timer_id) {
        timer = &epos_timer;
    } else if (TTC_TIMER_INITIAL_CHECKUP_SEQUENCE == timer_id) {
        timer = &initial_checkup_sequence_timer;
    } else {
        return;
    }
    u32 interrupt_status = XTtcPs_GetInterruptStatus(timer);
    XTtcPs_ClearInterruptStatus(timer, interrupt_status);
    XTtcPs_Stop(timer);
    XTtcPs_ResetCounterValue(timer);
}
