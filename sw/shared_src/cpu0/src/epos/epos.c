/*
 * epos.c
 *
 *  Created on: 26 Feb 2022
 *      Author: DANKLORD420
 */

#include "epos.h"
#include "../utility/can_types.h"
#include "../timer/ttc_timer.h"
#include "../utility/Revolve_CAN_definitions/Revolve_CAN_definitions.h"
#include <math.h>
#include "../zcan/zcan.h"

#define DEG2RAD(x)                 ((M_PI * x) / 180.0F)

#define MAX_STEERING_WHEEL_ANGLE   DEG2RAD(100)                // [rad]
#define MIN_STEERING_WHEEL_ANGLE   (-MAX_STEERING_WHEEL_ANGLE) // [rad]

#define INCREMENTS_PER_RAD_ENCODER 43025 // Use if the digital incremental encoder is used as main sensor.
#define EPOS_NMT_NODE_ID           0x01  // Specifies the NMT NODE-ID, which is hopefully set to 1 by the DIP switch of the EPOS4.
#define EPOS_OPERATIONAL           0x01

// clang-format off
static float steering_wheel_angle[21] = {
    -105.000, -94.500, -84.000, -73.500,
    -63.000, -52.500, -42.000, -31.500,
    -21.000, -10.500, 0.000, 10.500,
    21.000, 31.500, 42.000, 52.500,
    63.000, 73.500, 84.000, 94.500,
    105.000 };

static float steering_wheel_angle_per_wheel_angle[21] = {
    3.657, 3.703, 3.742, 3.774,
    3.801, 3.823, 3.840, 3.854,
    3.863, 3.869, 3.869, 3.869,
    3.863, 3.854, 3.840, 3.823,
    3.801, 3.774, 3.742, 3.703,
    3.657 };
// clang-format on

typedef enum {
    State_Not_Operational_1,
    State_Not_Operational_2,
    State_Operational_And_Configured
} epos_state_t;

static epos_state_t current_epos_state          = State_Not_Operational_1;
static float        steering_wheel_angle_offset = 0;
static float        actuator_increment_offset   = 0;

static Can_message_t epos_pdo1;
static Can_message_t epos_pdo2;
static Can_message_t epos_pdo3;
static Can_message_t epos_nmt_control_message;

static float epos_convert_to_steering_wheel_angle(float setpoint);
static bool  offset_is_set = false;
static bool  epos_active   = false;

void epos_init(void) {
    if (epos_active == true) {
        return;
    }
    zcan_can_tx(epos_nmt_control_message);
    current_epos_state = State_Not_Operational_1;
    ttc_timer_schedule_interrupt(1000, TTC_TIMER_EPOS);
}

void epos_deinitialize() {
    epos_active   = false;
    offset_is_set = false;
}

void epos_set_active() {
    epos_active = true;
}

bool epos_is_alive() {
    return epos_active;
}

void epos_send_setpoint(float steering_angle_setpoint) {
    if (epos_active == false) {
        return;
    }
    float steering_wheel_angle_setpoint = epos_convert_to_steering_wheel_angle(-steering_angle_setpoint);
    steering_wheel_angle_setpoint += steering_wheel_angle_offset;
    steering_wheel_angle_setpoint *= INCREMENTS_PER_RAD_ENCODER;
    steering_wheel_angle_setpoint += actuator_increment_offset;

    int steering_setpoint = (int)steering_wheel_angle_setpoint;
    epos_pdo3.data.i8[2]  = steering_setpoint;
    epos_pdo3.data.i8[3]  = steering_setpoint >> 8;
    epos_pdo3.data.i8[4]  = steering_setpoint >> 16;
    epos_pdo3.data.i8[5]  = steering_setpoint >> 24;
    epos_pdo3.data.u16[0] = 0x2F;
    zcan_can_tx(epos_pdo3);
    ttc_timer_schedule_interrupt(2000, TTC_TIMER_EPOS);
}

void epos_set_steering_wheel_angle_offset(float steering_wheel_angle) {
    if (offset_is_set) {
        return;
    }
    steering_wheel_angle_offset = steering_wheel_angle;
}

void epos_set_actuator_increment_offset(float increment_offset) {
    if (offset_is_set) {
        return;
    }
    actuator_increment_offset = increment_offset;
    offset_is_set             = true;
}

static float epos_convert_to_steering_wheel_angle(float setpoint) {
    setpoint = setpoint * 3.869;
    if (setpoint > MAX_STEERING_WHEEL_ANGLE) {
        setpoint = MAX_STEERING_WHEEL_ANGLE;
    } else if (setpoint < MIN_STEERING_WHEEL_ANGLE) {
        setpoint = MIN_STEERING_WHEEL_ANGLE;
    }
    return setpoint;
}

void epos_callback(void * ref) {
    ttc_timer_stop(TTC_TIMER_EPOS);

    if (current_epos_state == State_Not_Operational_1) {
        epos_pdo1.data.u16[0] = 0x0006;
        zcan_can_tx(epos_pdo1);
        current_epos_state = State_Not_Operational_2;
        ttc_timer_schedule_interrupt(1000, TTC_TIMER_EPOS);
    } else if (current_epos_state == State_Not_Operational_2) {
        epos_pdo1.data.u16[0] = 0x000F;
        zcan_can_tx(epos_pdo1);
        current_epos_state = State_Operational_And_Configured;
        ttc_timer_schedule_interrupt(1000, TTC_TIMER_EPOS);
    } else if (current_epos_state == State_Operational_And_Configured) {
        epos_pdo3.data.u16[0] = 0x3F;
        zcan_can_tx(epos_pdo3);
    }
}

static Can_message_t epos_pdo1 = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 2,
    .messageID  = EPOS_RECEIVE_PDO1
};

static Can_message_t epos_pdo3 = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 6,
    .messageID  = EPOS_RECEIVE_PDO3,
    .data.u64   = 0
};

static Can_message_t epos_nmt_control_message = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 2,
    .messageID  = DV_NMT_NODE_CONTROL,
    .data.u8[0] = EPOS_OPERATIONAL,
    .data.u8[1] = EPOS_NMT_NODE_ID
};
