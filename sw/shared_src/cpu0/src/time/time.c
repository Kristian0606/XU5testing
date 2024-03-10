/**
 * @file time.c
 * @author Sivaranjith Sivarasa
 * @brief for getting time since the start of the program
 * @version 0.1
 * @date 2021-05-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "time.h"
#include "xtime_l.h"

#define COUNTS_PER_MILLI_SECOND (COUNTS_PER_SECOND / 1000)
#define COUNTS_PER_MICRO_SECOND (COUNTS_PER_MILLI_SECOND / 1000)

void get_time_ms(u64 * timestamp) {
    XTime tCur = 0;
    XTime_GetTime(&tCur);
    *timestamp = (tCur / COUNTS_PER_MILLI_SECOND);
}

void get_time_us(volatile u64 * timestamp) {
    XTime tCur = 0;
    XTime_GetTime(&tCur);
    *timestamp = (tCur / COUNTS_PER_MICRO_SECOND);
}
