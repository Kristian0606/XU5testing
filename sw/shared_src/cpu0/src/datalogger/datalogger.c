/*
 * datalogger.c
 *
 *  Created on: 5 Mar 2022
 *      Author: DANKLOAD420
 */

#include "datalogger.h"
#include "../utility/can_types.h"
#include "../zcan/zcan.h"

static Can_message_t dv_driving_dynamics_1 = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 8,
    .messageID  = pu_DVDrivingDynamics1_1_0_CAN_ID,
};

static Can_message_t dv_driving_dynamics_2 = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 6,
    .messageID  = pu_DVDrivingDynamics2_1_0_CAN_ID,
};

static Can_message_t dv_system_status = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 5,
    .messageID  = pu_DVSystemStatus_1_0_CAN_ID,
};

// FSG
static Can_message_t hydraulic_pressure_readings = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 6,
    .messageID  = 0x511
};

static Can_message_t pneumatic_pressure_readings = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 4,
    .messageID  = 0x512
};

static Can_message_t sdc_logic_monitoring = {
    .idType     = STANDARD_CAN_ID,
    .dataLength = 1,
    .messageID  = 0x513
};

typedef struct dv_system_status_data {
    uint64_t AS_state : 3;
    uint64_t EBS_state : 2;
    uint64_t AMI_state : 3;
    uint64_t Steering_state : 1;
    uint64_t Service_brake_state : 2;
    uint64_t Lap_counter : 4;
    uint64_t Cones_count_actual : 8;
    uint64_t Cones_count_all : 17;
} dv_system_status_data_t;

typedef union dv_system_status {
    dv_system_status_data_t data_fields;
    uint64_t                data_raw;
} dv_system_status_t;

void parse_and_send_dv_driving_dynamics_1(pu_DVDrivingDynamics1_1_0 * msg) {
    dv_driving_dynamics_1.data.u8[0] = msg->speed_actual;
    dv_driving_dynamics_1.data.u8[1] = msg->speed_target;
    dv_driving_dynamics_1.data.i8[2] = msg->steering_angle_actual;
    dv_driving_dynamics_1.data.i8[3] = msg->steering_angle_target;
    dv_driving_dynamics_1.data.u8[4] = msg->brake_hydr_actual;
    dv_driving_dynamics_1.data.u8[5] = msg->brake_hydr_target;
    dv_driving_dynamics_1.data.i8[6] = msg->motor_moment_actual;
    dv_driving_dynamics_1.data.i8[7] = msg->motor_moment_target;
    zcan_can_tx(dv_driving_dynamics_1);
}

void parse_and_send_dv_driving_dynamics_2(pu_DVDrivingDynamics2_1_0 * msg) {
    dv_driving_dynamics_2.data.i16[0] = msg->acceleration_longitudinal;
    dv_driving_dynamics_2.data.i16[1] = msg->acceleration_lateral;
    dv_driving_dynamics_2.data.i16[2] = msg->yaw_rate;
    zcan_can_tx(dv_driving_dynamics_2);
}

void parse_and_send_dv_system_status(pu_DVSystemStatus_1_0 * msg) {
    dv_system_status_t dv_status;
    dv_status.data_fields.AS_state            = msg->as_state;
    dv_status.data_fields.EBS_state           = msg->ebs_state;
    dv_status.data_fields.AMI_state           = msg->ami_state;
    dv_status.data_fields.Steering_state      = msg->steering_state;
    dv_status.data_fields.Service_brake_state = msg->service_brake_state;
    dv_status.data_fields.Lap_counter         = msg->lap_counter;
    dv_status.data_fields.Cones_count_actual  = msg->cones_count_actual;
    dv_status.data_fields.Cones_count_all     = msg->cones_count_all;

    dv_system_status.data.u64 = dv_status.data_raw;
    zcan_can_tx(dv_system_status);
}

void datalogger_transmit_hydraulic_pressure_readings(float hydraulic_pressure_front, float hydraulic_pressure_rear) {
    const float BAR_TO_MBAR    = 1000;
    u32         pressure_front = hydraulic_pressure_front * BAR_TO_MBAR;
    u32         pressure_rear  = hydraulic_pressure_rear * BAR_TO_MBAR;

    hydraulic_pressure_readings.data.u8[0] = pressure_front >> 0;
    hydraulic_pressure_readings.data.u8[1] = pressure_front >> 8;
    hydraulic_pressure_readings.data.u8[2] = pressure_front >> 16;

    hydraulic_pressure_readings.data.u8[3] = pressure_rear >> 0;
    hydraulic_pressure_readings.data.u8[4] = pressure_rear >> 8;
    hydraulic_pressure_readings.data.u8[5] = pressure_rear >> 16;

    zcan_can_tx(hydraulic_pressure_readings);
}

void datalogger_transmit_pneumatic_pressure_readings(float pneumatic_pressure_front, float pneumatic_pressure_rear) {
    const float BAR_TO_MBAR  = 1000;
    u16         pressure_EBS = pneumatic_pressure_front * BAR_TO_MBAR;
    u16         pressure_ASB = pneumatic_pressure_rear * BAR_TO_MBAR;

    pneumatic_pressure_readings.data.u16[0] = pressure_EBS;
    pneumatic_pressure_readings.data.u16[1] = pressure_ASB;

    zcan_can_tx(pneumatic_pressure_readings);
}

void datalogger_transmit_sdc_logic_monitoring(bool ASMS_on,
                                              bool as_close_SDC,
                                              bool as_driving_mode,
                                              bool watchdog,
                                              bool SDC_is_ready) {

    // clang-format off
    sdc_logic_monitoring.data.u8[0] = (ASMS_on         << 0) |
                                      (as_close_SDC    << 1) |
                                      (as_driving_mode << 2) |
                                      (watchdog        << 3) |
                                      (SDC_is_ready    << 4);
    // clang-format on

    zcan_can_tx(sdc_logic_monitoring);
}
