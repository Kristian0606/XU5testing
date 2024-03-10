//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/timer/timer.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 16-04-19
#ifndef SRC_TIMER_PS_TIMER_H_
#define SRC_TIMER_PS_TIMER_H_

#include "../utility/types.h"

/**
 * @brief Initializes the PS timer.
 * 
 */
void ps_timer_init();

/**
 * @brief Starts the PS timer.
 * 
 * @param msec the time in milliseconds until the interrupt should be triggered
 */
void ps_timer_start(u32 msec);

/**
 * @brief Stops the PS timer.
 * 
 */
void ps_timer_stop();

#endif /* SRC_TIMER_PS_TIMER_H_ */
