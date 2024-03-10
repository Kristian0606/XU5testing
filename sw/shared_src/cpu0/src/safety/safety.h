//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/safety/safety.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 15-05-19
#ifndef SRC_SAFETY_SAFETY_H_
#define SRC_SAFETY_SAFETY_H_
#include "../utility/types.h"
#include "../timer/axi_timer.h"

#define SET                            0x01
#define CLEAR                          0x00

#define MANUAL_DE_BRK_LIM              5.0F // bar

#define AUTONOMOUS_PNEUMATIC_LOWER_LIM 3.0F // bar

#define ABPI_TIMEOUT                   400U    /*  ms */
#define ABPI_PWR_LIM                   5000.0F /*   W */
#define ABPI_ACC_LIM                   0.25F   /*   % */
#define ABPI_BRK_LIM                   15.0F   /* bar */

#define API_TIMEOUT                    100U /*  ms */
#define API_DIFF_LIM                   0.1F /*   % */

#define SAFE                           1U
#define UNSAFE                         0U

void start_abpi_timer();
void start_api_timer();
void stop_abpi_timer();
void stop_api_timer();

void safety_start_initial_checkup_sequence(vcu_data_t * vcu_data_ptr);
void safety_initial_checkup_sequence_callback(void * ref);
void safety_reset_initial_checkup_sequence(void);
bool safety_check_as_power_limit(u16 rpm, u16 trq);
bool safety_check_as_inspection_mission_power_limit(u16 rpm, u16 trq);
bool safety_check_as_kers_limit(u16 rpm, u16 trq);
bool safety_check_as_inspection_mission_kers_limit(u16 rpm, u16 trq);
bool safety_check_new_drive_mode(u8 mission, u8 new_mode);
u8   safety_check_brake_pressure(vcu_data_t * vcu_data_p);
u8   safety_acceleration_brake_pedal_plausibility(f32 brakes, f32 throttle, f32 pwr);
u8   safety_acceleration_pedal_plausibility(f32 apps1, f32 apps2);
void safety_continuous_monitoring(vcu_data_t * vcu_data_p, bool as_driving);
f32  get_throttle(f32 apps1, f32 apps2);
f32  get_min_throttle(f32 apps1, f32 apps2);
void safety_update_pu_alive(void);
void safety_update_epos_alive(void);
void safety_update_lidar_alive(bool lidar_alive);
void safety_update_pu_inverter_setpoint(vcu_data_t * vcu_data_p);
void safety_update_pu_tv_setpoint(vcu_data_t * vcu_data_p);
void safety_update_pu_launch_setpoint(vcu_data_t * vcu_data_p);
void safety_res_emergency(bool emergency);
void safety_pu_requests_ebs(void);

#endif /* SRC_SAFETY_SAFETY_H_ */
