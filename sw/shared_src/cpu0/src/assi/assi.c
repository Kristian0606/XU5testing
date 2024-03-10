/*
 * assi.c
 *
 *  Created on: 4 Apr 2022
 *      Author: Audun Amadeus Olsen
 */

#include "assi.h"
#include "../gpio/gpio.h"

void handle_assi(assi_state state) {
    if (AS_off == state) {
        write_gpio_pin(ASSI_YELLOW, 0);
        write_gpio_pin(ASSI_BLUE, 0);
    } else if (AS_ready == state) {
        write_gpio_pin(ASSI_YELLOW, 1);
        write_gpio_pin(ASSI_BLUE, 0);
    } else if (AS_driving == state) {
        toggle_gpio_pin(ASSI_YELLOW);
        write_gpio_pin(ASSI_BLUE, 0);
    } else if (AS_emergency == state) {
        write_gpio_pin(ASSI_YELLOW, 0);
        toggle_gpio_pin(ASSI_BLUE);
    } else if (AS_finished == state) {
        write_gpio_pin(ASSI_YELLOW, 0);
        write_gpio_pin(ASSI_BLUE, 1);
    }
}
