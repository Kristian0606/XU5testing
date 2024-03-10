//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/utility/types.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 17-04-19
#ifndef SRC_UTILITY_TYPES_H_
#define SRC_UTILITY_TYPES_H_

#include "xil_types.h"
#include "canard.h"

#include "../shared_src/utility/shared_types.h"

typedef enum {
    INV_TYPE_I19,
    INV_TYPE_I21,
} inverter_type_t;

typedef enum {
    FL             = 0U,
    FR             = 1U,
    RL             = 2U,
    RR             = 3U,
    INVERTER_COUNT = 4
} inverter_t;

#define GRANTED 1
#define DENIED  0

/*  Inverter Control Message:
 *  Message defined as inv_ctr_t msg[4] = {FL, FR, RL, RR}
 *  One message for all inverters.
 */
#define RP_INV_CTRL_ID  0x170
#define RP_INV_CTRL_LEN 32U

#define INV_CMD_BLANK    0x00
#define INV_CMD_ENABLE   (0xFFFF & (1 << 0))
#define INV_CMD_DISABLE  (0xFFFF & (1 << 1))
#define INV_CMD_RESET    (0xFFFF & (1 << 5))
#define INV_CMD_FW       (0xFFFF & (1 << 8))
#define INV_CMD_SETPOINT (0xFFFF & (1 << 9))

typedef struct {
    u16 cmd;
    u16 rpm_req;
    u16 trq_max;
    u16 trq_min;
} inv_ctrl_t;

/*  Inverter Feedback Message:
 *  One message from each inverter.
 */
#define RP_INV_FEEDBACK_FL_ID 0x270
#define RP_INV_FEEDBACK_FR_ID 0x274
#define RP_INV_FEEDBACK_RL_ID 0x278
#define RP_INV_FEEDBACK_RR_ID 0x27C

typedef struct {
    f32 enc_angle;
    f32 enc_rpm;
    f32 trq_used;
    f32 trq_measured;
    f32 rpm_req;
} rp_inv_feedback_t;

/*  Inverter Status Message:
 *  One message from each inverter.
 */
#define RP_INV_STATUS_FL_ID 0x273
#define RP_INV_STATUS_FR_ID 0x277
#define RP_INV_STATUS_RL_ID 0x27B
#define RP_INV_STATUS_RR_ID 0x27F

#define INV_STATE_STARTUP   (0xFF & (1 << 0))
#define INV_STATE_IDLE      (0xFF & (1 << 1))
#define INV_STATE_READY     (0xFF & (1 << 2))
#define INV_STATE_ERROR     (0xFF & (1 << 3))

typedef struct {
    u32 status;
    u32 encoder;
    u8  state;
} rp_inv_status_t;

typedef struct {
    u8 data;
} remote_emergency_system_pre_operational_t;

typedef struct {
    u8 data[8];
} remote_emergency_system_operational_t;

typedef struct {
    u16 error_code;
    u8  modes_of_operation_display;
    int following_error;
} __attribute__((__packed__)) epos_pdo1_t;

typedef struct {
    int position;
    int current;
} __attribute__((__packed__)) epos_pdo2_t;

typedef struct {
    int16_t torque;
    int     velocity;
    u16     status_word;
} __attribute__((__packed__)) epos_pdo3_t;

typedef struct {
    f32 vx;
    f32 vy;
    f32 vz;
    f32 ax;
    f32 ay;
    f32 az;
    f32 roll;
    f32 pitch;
    f32 yaw;
    f32 roll_rate;
    f32 pitch_rate;
    f32 yaw_rate;
    f32 roll_rate_dt;
    f32 pitch_rate_dt;
    f32 yaw_rate_dt;
} ins_data_t;

typedef struct {
    f32 apps_l;
    f32 apps_r;
    f32 yaw;
    f64 lat;
    f64 lon;
    f64 alt;
    u8  gnss_a_sats;
    u8  gnss_a_fix;
    u8  status_time;
    u16 status_ins;
    u8  gnss_b_sats;
    u8  gnss_b_fix;

    u32 time_gps_msb;
    u32 time_gps_lsb;
    f32 ax_raw;
    f32 ay_raw;
    f32 az_raw;
    f32 roll_rate_raw;
    f32 pitch_rate_raw;
    f32 yaw_rate_raw;
    u32 time_gps_pps_msb;
    u32 time_gps_pps_lsb;
    u8  time_status;
    f32 yaw_std;
    f32 pitch_std;
    f32 roll_std;
    f32 pos_std;
    f32 vel_std;
    u16 ahrs_status;
} meta_data_t;

typedef struct {
    u32          rx_flag;
    u16          ins_rx_cnt;
    u16          ins_crc_er;
    u16          ev_cnt;
    u8           tv_cfg_busy;
    u8           abpp;
    u8           abpi;
    u16          abpi_cnt;
    u8           app;
    u8           api;
    u16          api_cnt;
    cpu_status_t cpu0;
    cpu_status_t cpu1;
} status_t;

typedef struct {
    u8  mode;
    u8  mission;
    f32 tv_max_trq;
    f32 tv_min_trq;
    f32 tv_max_rpm;

    f32 st_trq;
    f32 st_rpm;

    f32 pitch_zero;
    f32 roll_zero;
} config_t;

typedef struct {
    u8  status;
    u16 numMes;
    f32 posX;
    f32 posY;
    f32 posZ;
    f32 unX;
    f32 unY;
    f32 unZ;
} ins_calib_t;

/* Types of SCS */
// NB: This is f***ing magic as always. The higher the number, the higher the priority.
// Should be fixed in another way
typedef enum {
    SCS_OK          = 0x00,
    SCS_AS_CRIT     = 0x01, // Critical during autonomous driving
    SCS_TV_CRIT     = 0x02, // Critical during TV
    SCS_SBS_TV_CRIT = 0x03, // Critical during TV, but signals marking error from SBS
    SCS_SBS_CRIT    = 0x04, // Critical during Drive Enable, but signals marking error from SBS
    SCS_DE_CRIT     = 0x05, // Critical during Drive Enable
} scs_status_t;

typedef struct {
    f32          max;
    f32          min;
    u8           time_out;
    u8           time_cnt;
    u8           state;
    scs_status_t type;
} scs_t;

typedef enum {
    /* Inputs */
    SCS_APPS1,
    SCS_APPS2,
    SCS_HYDRAULIC_BRAKES_F,
    SCS_HYDRAULIC_BRAKES_R,
    SCS_PNEUMATIC_BRAKES_F,
    SCS_PNEUMATIC_BRAKES_R,
    SCS_KERS,
    SCS_STEERING,
    SCS_STEERING_DT,
    SCS_VOLT,
    SCS_MAX_CELL_VOLT,
    SCS_POWER,
    SCS_POWER_DT,
    SCS_VX,
    SCS_VY,
    SCS_VZ,
    SCS_AX,
    SCS_AY,
    SCS_AZ,
    SCS_YAW_DT,
    SCS_YAW_DTDT,
    SCS_ROLL,
    SCS_ROLL_DT,
    SCS_PITCH,
    SCS_PITCH_DT,
    SCS_TRQ_FL,
    SCS_TRQ_FR,
    SCS_TRQ_RL,
    SCS_TRQ_RR,
    SCS_RPM_FL,
    SCS_RPM_FR,
    SCS_RPM_RL,
    SCS_RPM_RR,
    /* Outputs */
    SCS_TV_TRQ_MIN_FL,
    SCS_TV_TRQ_MIN_FR,
    SCS_TV_TRQ_MIN_RL,
    SCS_TV_TRQ_MIN_RR,
    SCS_TV_TRQ_MAX_FL,
    SCS_TV_TRQ_MAX_FR,
    SCS_TV_TRQ_MAX_RL,
    SCS_TV_TRQ_MAX_RR,
    SCS_TV_RPM_FL,
    SCS_TV_RPM_FR,
    SCS_TV_RPM_RL,
    SCS_TV_RPM_RR,
    /* PU setpoints */
    SCS_PU_TRQ_MIN_FL,
    SCS_PU_TRQ_MIN_FR,
    SCS_PU_TRQ_MIN_RL,
    SCS_PU_TRQ_MIN_RR,
    SCS_PU_TRQ_MAX_FL,
    SCS_PU_TRQ_MAX_FR,
    SCS_PU_TRQ_MAX_RL,
    SCS_PU_TRQ_MAX_RR,
    SCS_PU_RPM_FL,
    SCS_PU_RPM_FR,
    SCS_PU_RPM_RL,
    SCS_PU_RPM_RR,
    SCS_PU_STEERING_SETPOINT,
    SCS_PU_FX_SETPOINT,
    SCS_PU_MZ_SETPOINT,
    SCS_PU_LAUNCH_SETPOINT,
    /* SBS SCS Signals */
    SCS_SBS_APPS1,
    SCS_SBS_APPS2,
    SCS_SBS_HYDRAULIC_BRAKES_F,
    SCS_SBS_HYDRAULIC_BRAKES_R,
    SCS_SBS_PNEUMATIC_BRAKES_F,
    SCS_SBS_PNEUMATIC_BRAKES_R,
    /* SBS SCS TV Signals */
    SCS_SBS_KERS,
    SCS_SBS_STEERING,

    /* Number of SCS */
    /* ALWAYS keep at bottom of enum, used to define array size */
    SCS_NUM
} scs_types_t;

typedef struct {
    u8 message_nr;
    u8 buffer_filled;
} telemetry_t;

typedef struct {
    u16   status_word;
    u16   error_code;
    u8    modes_of_operation_display;
    float position;
    float velocity;
    float torque;
    float current;
    float following_error;
} epos_data_t;

typedef struct {
    tv_input_t       tv_input;
    tv_output_t      tv_output;
    tv_config_t      tv_config;
    CanardTransferID tv_config_destination_id;
    ins_data_t       ins_data;
    meta_data_t      meta_data;
    inv_ctrl_t       inv_sp[4];
    inv_setpoints_t  pu_setpoints;
    float            pu_steering_setpoint;
    bool             pu_launch_trigger;
    f32              pu_target_fx;
    f32              pu_target_mz;
    u8               inv_state[4];
    u8               ams_state;
    config_t         config;
    status_t         status;
    ins_calib_t      ins_calib;
    scs_t            scs[SCS_NUM];
    u8               scs_errors;
    telemetry_t      telemetry;
    epos_data_t      epos_data;
    inverter_type_t  inverter_type;
} vcu_data_t;

/*
 * Energy meter status message
 */
#define ENERGY_METER_ID 0x430

#endif /* SRC_UTILITY_TYPES_H_ */
