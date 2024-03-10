//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/timer/timer.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 16-04-19
#include "ps_timer.h"

#include "xparameters.h"
#include "xtime_l.h"


/*
 * Likely due to rounding errors, timers are slightly off.
 * By setting 1s to be 1004 ms, the error is compensated for.
 */
#define MSECS_PER_SEC 1000U

#define PRESCALER     0xFF
#define F_TIMER       XPAR_PSU_CORTEXA53_0_CPU_CLK_FREQ_HZ

static u64 timer;
static u64 limit;


void ps_timer_start(u32 msec) {
	XTime_GetTime(&timer);
	limit = timer + msec*F_TIMER; //some maths needed
}

void ps_timer_check(){
	if (limit <= timer){
		//Make an interupt or smth
	} else {
		XTime_GetTime(&timer);
	}
}

