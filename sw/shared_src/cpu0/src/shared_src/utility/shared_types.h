//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/utility/tv_types.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 14-05-19
#ifndef SRC_UTILITY_SHARED_TYPES_H_
#define SRC_UTILITY_SHARED_TYPES_H_

#include "xil_types.h"

#include "vcu/State_1_0.h"
#include "vcu/Warning_1_0.h"
#include "tv/LaunchControl_1_0.h"
#include "tv/PowerLimiting_1_0.h"
#include "tv/QpDebug_1_0.h"
#include "tv/ForceSetpoints_1_0.h"
#include "tv/MzRefDebug_1_0.h"
#include "tv/TyreForces_1_0.h"
#include "tv/TyreSlip_1_0.h"
#include "tv/TyreState_1_0.h"
#include "tv/VehicleState_1_0.h"

/* Headers for tv_config_t: */
#include "tv/params/General_1_0.h"
#include "tv/params/PowerLimitingAcc_1_0.h"
#include "tv/params/PowerLimitingKERS_1_0.h"
#include "tv/params/StateEstimation_1_0.h"
#include "tv/params/RegularControl_1_0.h"
#include "tv/params/YMR_1_0.h"
#include "tv/params/YMRGains_1_0.h"
#include "tv/params/Launch_1_0.h"
#include "tv/params/TyreTuning_1_0.h"
#include "tv/params/vxControl_1_0.h"
#include "tv/params/modeToggle_1_0.h"
#include "tv/params/pedalMap_1_0.h"
#include "tv/params/YMRWeights_1_0.h"

typedef float  f32;
typedef double f64;

typedef struct {
    u8  state;
    u8  load;
    u16 ev_hz;
    u8  ev_buf;
} cpu_status_t;

/*
 * TV Input:
 */
typedef struct {
    f32 throttle;
    f32 hydraulic_brake_front;
    f32 hydraulic_brake_rear;
    f32 pneumatic_brake_front;
    f32 pneumatic_brake_rear;
    f32 KERS;
    f32 steering_angle;
    f32 steering_rate;
    f32 vx;
    f32 vy;
    f32 vz;
    f32 ax;
    f32 ay;
    f32 az;
    f32 yaw_rate;
    f32 yaw_rate_dt;
    f32 roll;
    f32 roll_rate;
    f32 pitch;
    f32 pitch_rate;
    f32 RPM_FL;
    f32 RPM_FR;
    f32 RPM_RL;
    f32 RPM_RR;
    f32 power;
    f32 DC_voltage;
    f32 max_cell_voltage;
    f32 torque_FL;
    f32 torque_FR;
    f32 torque_RL;
    f32 torque_RR;
    f32 power_dt;
    f32 as_Fx_requested;
    f32 as_Mz_requested;
    f32 as_driving_mode;
    /* Flag to tell TV if INS is usable */
    u32 ins_aligned;
} tv_input_t;

/*
 * TV Output:
 */
typedef struct {
    /* Front Left  */
    f32 rpm_req_fl;
    f32 trq_max_fl;
    f32 trq_min_fl;
    /* Front Right */
    f32 rpm_req_fr;
    f32 trq_max_fr;
    f32 trq_min_fr;
    /* Rear Left   */
    f32 rpm_req_rl;
    f32 trq_max_rl;
    f32 trq_min_rl;
    /* Rear Right  */
    f32 rpm_req_rr;
    f32 trq_max_rr;
    f32 trq_min_rr;
} inv_setpoints_t;

/*
 * TV Messages:
 */
typedef struct {
    tv_TyreForces_1_0 tyre_forces;
    tv_TyreSlip_1_0   tyre_slips;
    tv_TyreState_1_0  tyre_states;
} se_tyre_state_t;

typedef struct {
    tv_VehicleState_1_0 vehicle_state;
} se_vehicle_state_t;

typedef struct {
    tv_LaunchControl_1_0 launch_control;
} tv_launch_control_t;

typedef struct {
    tv_PowerLimiting_1_0  power_limiting;
    tv_QpDebug_1_0        qp_debug;
    tv_ForceSetpoints_1_0 force_setpoints;
    tv_MzRefDebug_1_0     mz_ref_debug;
} tv_regular_control_t;

typedef struct {
    u32 exec_t;
    u8  state;
    u8  status;
} tv_status_t;

typedef struct {
    inv_setpoints_t      inv_sp;
    se_tyre_state_t      se_tyres;
    se_vehicle_state_t   se_vehicle;
    tv_regular_control_t tv_regular_control;
    tv_launch_control_t  tv_launch_control;
    tv_status_t          status;
} tv_output_t;

/*
 * TV Configuration:
 */
#define CONFIG_GET_ALL             0xFFFFFFFF
#define CONFIG_GENERAL             (1 << 0)
#define CONFIG_POWER_LIMITING_ACC  (1 << 1)
#define CONFIG_POWER_LIMITING_KERS (1 << 2)
#define CONFIG_REGULAR_CONTROL     (1 << 3)
#define CONFIG_YMR                 (1 << 4)
#define CONFIG_YMR_GAINS           (1 << 5)
#define CONFIG_LAUNCH              (1 << 6)
#define CONFIG_TYRE_TUNING         (1 << 7)
#define CONFIG_VX_CONTROL          (1 << 8)
#define CONFIG_MODE_TOGGLE         (1 << 9)
#define CONFIG_STATUS              (1 << 10)
#define CONFIG_PEDAL_MAP           (1 << 11)
#define CONFIG_STATE_ESTIMATION    (1 << 12)
#define CONFIG_YMR_WEIGHTS         (1 << 13)

typedef struct {
    u32                                     bfield;
    tv_params_General_Request_1_0           general;
    tv_params_PowerLimitingAcc_Request_1_0  power_limiting_acc;
    tv_params_PowerLimitingKERS_Request_1_0 power_limiting_kers;
    tv_params_StateEstimation_Request_1_0   state_estimation;
    tv_params_RegularControl_Request_1_0    regular_control;
    tv_params_YMR_Request_1_0               ymr;
    tv_params_YMRGains_Request_1_0          ymr_gains;
    tv_params_Launch_Request_1_0            launch;
    tv_params_TyreTuning_Request_1_0        tyre_tuning;
    tv_params_vxControl_Request_1_0         vx_control;
    tv_params_modeToggle_Request_1_0        mode_toggle;
    tv_params_pedalMap_Request_1_0          pedal_map;
    tv_params_YMRWeights_Request_1_0        ymr_weights;
} tv_config_t;

#endif /* SRC_UTILITY_SHARED_TYPES_H_ */
