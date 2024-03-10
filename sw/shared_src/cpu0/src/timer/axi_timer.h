//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/timer/axi_timer.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 04-06-19
#ifndef SRC_TIMER_AXI_TIMER_H_
#define SRC_TIMER_AXI_TIMER_H_

#include "../shared_src/intc/intc.h"

typedef enum {
    TIMER_SAFETY_ABPI = 0,
    TIMER_SAFETY_API,
    TIMER_INV,
    TIMER_INV_UNUSED
} axi_timer_t;


/**
 * @brief Initializes the AXI timer.
 * 
 * @param isr0 pointer to the first interrupt service routine
 * @param isr1 pointer to the second interrupt service routine
 * @param intc_p pointer to the interrupt controller instance
 */
void axi_timer_init(void * isr0, void * isr1, XScuGic * intc_p);

/**
 * @brief Starts the AXI timer.
 * 
 * @param id the id of the timer to start
 * @param msec the time in milliseconds until the interrupt should be triggered
 */
void axi_timer_start(axi_timer_t id, u32 msec);

/**
 * @brief Stops the AXI timer.
 * 
 * @param id the id of the timer to stop
 */
void axi_timer_stop(axi_timer_t id);


/**
 * @brief Returns the index of the AXI timer.
 * 
 * @param timer the timer to get the index of
 * @return u8 the index of the timer
 */
static inline u8 axi_timer_idx(axi_timer_t timer) {
    // need to have implementation in header because it's inline
    switch (timer) {
    default: // smh
    case TIMER_SAFETY_ABPI:
    case TIMER_INV:
        return 0;
    case TIMER_SAFETY_API:
    case TIMER_INV_UNUSED:
        return 1;
    }
}

#endif /* SRC_TIMER_AXI_TIMER_H_ */
