/*
 * epos.h
 *
 *  Created on: 26 Feb 2022
 *      Author: DANKLORD420
 */

#ifndef SRC_EPOS_EPOS_H_
#define SRC_EPOS_EPOS_H_

#include <stdbool.h>

// Configurable in epos studio. Epos is the recieving end.
#define EPOS_RECEIVE_PDO1    0x300
#define EPOS_RECEIVE_PDO2    0x301
#define EPOS_RECEIVE_PDO3    0x302
#define EPOS_RECEIVE_PDO4    0x303
#define EPOS_TRANSMIT_PDO1   0x400
#define EPOS_TRANSMIT_PDO2   0x401
#define EPOS_TRANSMIT_PDO3   0x402
#define EPOS_TRANSMIT_PDO4   0x403

#define EPOS_TORQUE_CONSTANT 327.0 // Nominal torque [mNm]

void epos_init();
void epos_deinitialize();
void epos_send_setpoint(float steering_angle_setpoint);
void epos_set_steering_wheel_angle_offset(float steering_wheel_angle);
void epos_set_actuator_increment_offset(float increment_offset);
void epos_set_active();
bool epos_is_alive();
void epos_callback(void * ref);

#endif /* SRC_EPOS_EPOS_H_ */
