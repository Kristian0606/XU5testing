/**
 * @file time.h
 * @author Sivaranjith Sivarasa
 * @brief for getting time since the start of the program
 * @version 0.1
 * @date 2021-05-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef _ZTIME_H_
#define _ZTIME_H_

#include "xil_types.h"

/**
 * @brief Get the time in milliseconds since startup
 *
 * @param timestamp Pointer to u64 which will become the time in milliseconds since startup
 */
void get_time_ms(u64 * timestamp);

/**
 * @brief Get the time in microseconds since startup
 *
 * @param timestamp Pointer to u64 which will become the time in microseconds since startup
 */
void get_time_us(volatile u64 * timestamp);

#endif
