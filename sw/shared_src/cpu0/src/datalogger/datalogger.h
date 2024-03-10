/*
 * datalogger.h
 *
 *  Created on: 5 Mar 2022
 *      Author: DANKLOARD420
 */

// Implemented according to FSG22_Competition_Handbook_v1.1

#ifndef SRC_DATALOGGER_DATALOGGER_H_
#define SRC_DATALOGGER_DATALOGGER_H_

#include "pu/DVDrivingDynamics1_1_0.h"
#include "pu/DVDrivingDynamics2_1_0.h"
#include "pu/DVSystemStatus_1_0.h"

void parse_and_send_dv_driving_dynamics_1(pu_DVDrivingDynamics1_1_0 * msg);
void parse_and_send_dv_driving_dynamics_2(pu_DVDrivingDynamics2_1_0 * msg);
void parse_and_send_dv_system_status(pu_DVSystemStatus_1_0 * msg);
void datalogger_transmit_hydraulic_pressure_readings(float hydraulic_pressure_front, float hydraulic_pressure_rear);
void datalogger_transmit_pneumatic_pressure_readings(float pneumatic_pressure_front, float pneumatic_pressure_rear);
void datalogger_transmit_sdc_logic_monitoring(bool ASMS_on,
                                              bool as_close_SDC,
                                              bool as_driving_mode,
                                              bool watchdog,
                                              bool SDC_is_ready);

#endif /* SRC_DATALOGGER_DATALOGGER_H_ */
