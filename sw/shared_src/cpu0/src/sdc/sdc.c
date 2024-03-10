/*
 * sdc.c
 *
 *  Created on: 26 Mar 2022
 *      Author: DANKLOAD420
 */

#include "sdc.h"
#include "../gpio/gpio.h"
#include "stdbool.h"

static bool watchdog_enabled = false;

void sdc_init(void) {
    write_gpio_pin(ACTIVATE_LATCH, 0);
    write_gpio_pin(ACTIVATE_TS, 0);
}

void sdc_latch(void) {
    write_gpio_pin(ACTIVATE_LATCH, 1);
    write_gpio_pin(ACTIVATE_TS, 0);
}

void sdc_activate_ts(void) {
    write_gpio_pin(ACTIVATE_TS, 1);
}

void sdc_deactivate_ts(void) {
    write_gpio_pin(ACTIVATE_TS, 0);
}
void sdc_trigger_latch(void) {
    write_gpio_pin(ACTIVATE_LATCH, 1);
}

void sdc_try_unlatch(void) {
    write_gpio_pin(ACTIVATE_LATCH, 0);
}

void sdc_toggle_watchdog(void) {
    if (true == watchdog_enabled) {
        toggle_gpio_pin(WATCHDOG_OUT);
    }
}

bool sdc_is_latch_latched(void) {
    u32 latch_status = read_gpio_pin(LATCH_STATUS);
    if (latch_status == 0) {
        return true;
    }
    return false;
}

void sdc_enable_watchdog(void) {
    watchdog_enabled = true;
}

void sdc_disable_watchdog(void) {
    watchdog_enabled = false;
}

bool sdc_is_watchdog_good(void) {
    return !read_gpio_pin(WATCHDOG_STATUS);
}

bool sdc_is_ts_on(void) {
    return read_gpio_pin(SDC_STATUS);
}
