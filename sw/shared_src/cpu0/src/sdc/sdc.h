/*
 * datalogger.c
 *
 *  Created on: 26 Mar 2022
 *      Author: DANKLOAD420
 */
#ifndef SRC_SDC_SDC_H_
#define SRC_SDC_SDC_H_

#include "stdbool.h"

/** 
 * @brief Initializes the shutdown circuit.
*/
void sdc_init(void);
/** 
 * @brief Activates the shutdown circuit.
*/
void sdc_activate_ts(void);
/**
 * @brief Deactivates the shutdown circuit.
*/
void sdc_deactivate_ts(void);
/** 
 * @brief Triggers the shutdown circuit latch.
*/
void sdc_trigger_latch(void);
/** 
 * @brief Tries to unlatch the shutdown circuit.
*/
void sdc_try_unlatch(void);
/** 
 * @brief Checks if the shutdown circuit is latched.
 * @return True if latched, false if not. (bool)
*/
bool sdc_is_latch_latched(void);
/** 
 * @brief Toggles the watchdog.
*/
void sdc_toggle_watchdog(void);
/** 
 * @brief Enables the watchdog.
*/
void sdc_enable_watchdog(void);
/** 
 * @brief Disables the watchdog.
*/
void sdc_disable_watchdog(void);
/** 
 * @brief Checks if the watchdog is good.
 * @return True if good, false if not. (bool)
*/
bool sdc_is_watchdog_good(void);
/** 
 * @brief Checks if the shutdown circuit is on.
 * @return True if on, false if not. (bool)
*/
bool sdc_is_ts_on(void);

#endif /* SRC_SDC_SDC_H_ */
