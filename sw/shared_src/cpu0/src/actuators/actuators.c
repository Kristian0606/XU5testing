/*
 * actuators.c
 *
 *  Created on: 2 Apr 2022
 *      Author: DANKLOARD
 */

#include "actuators.h"
#include "../gpio/gpio.h"
#include "math.h"

#define HYDRAULIC_BRAKES_ENGAGED_THRESHOLD 3.0F // [bar]
static bool emergency_brakes_activated = false;

// checks that the EBS and ABS are (de)activated if the emergency brakes are (de)activated
//
// concretely:
// checks that EBS and ASB is activated if emergency brakes are ON,
// and that EBS and ASB is de-activated if emergency brakes are OFF
bool actuators_solenoid_ok(void) {
    // EBS and ASB are conceptually the same, but one is for front brakes and
    // one is for the back brakes (no, doesn't make a lot of sense)
    // EBS = Emergency Brake System
    // ASB = Autonomous System Brake
    bool ebs_status = read_gpio_pin(EBS_STATUS);
    bool asb_status = read_gpio_pin(ASB_STATUS);

    if (emergency_brakes_activated) {
        if (!ebs_status || !asb_status) {
            return false;
        }
    } else {
        if (ebs_status || asb_status) {
            return false;
        }
    }

    return true;
}

void actuators_activate_ebs(void) {
    write_gpio_pin(ASB_ENGAGE, 0);
    write_gpio_pin(EBS_ENGAGE, 0);
    emergency_brakes_activated = true;
}

void actuators_deactivate_ebs(void) {
    write_gpio_pin(ASB_ENGAGE, 1);
    write_gpio_pin(EBS_ENGAGE, 1);
    emergency_brakes_activated = false;
}

bool actuators_is_ebs_engaged() {
    return emergency_brakes_activated;
}

bool actuators_is_hydraulic_brakes_engaged(vcu_data_t * vcu_data_p) {
    f32 hydraulic_max = fmax(vcu_data_p->tv_input.hydraulic_brake_front, vcu_data_p->tv_input.hydraulic_brake_rear);
    if (hydraulic_max >= HYDRAULIC_BRAKES_ENGAGED_THRESHOLD) {
        return true;
    }
    return false;
}
