/*
 * ttc_timer.h
 *
 *  Created on: 28 Feb 2022
 *      Author: DANKLORD420
 */

#ifndef SRC_TIMER_TTC_TIMER_H_
#define SRC_TIMER_TTC_TIMER_H_

#include "xil_types.h"

#define TTC_TIMER_EPOS                     0
#define TTC_TIMER_INITIAL_CHECKUP_SEQUENCE 1
#define TTC_TIMER_3                        2


/**
 * @brief Initializes the TTC timer.
 */
void ttc_timer_init(void);

/**
 * @brief Schedules an interrupt for a TTC timer.
 * 
 * @param us the time in microseconds until the interrupt should be triggered
 * @param timer_id the id of the timer to schedule the interrupt on
 */
void ttc_timer_schedule_interrupt(int us, const u8 timer_id);

/**
 * @brief Stops a TTC timer.
 * 
 * @param timer_id the id of the timer to stop
 */
void ttc_timer_stop(const u8 timer_id);

#endif /* SRC_TIMER_TTC_TIMER_H_ */
