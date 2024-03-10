//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/safety/scs.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 21-06-19
#include "scs.h"
#include "math.h"
#include "../utility/types.h"

#define INF          INFINITY

#define SCS_OVER_SAT 0b00000001
#define SCS_UNDR_SAT 0b00000010
#define SCS_NAN      0b00000100
#define SCS_TIME_OUT 0b00001000

/* Inputs */
static const scs_t apps1              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t apps2              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t hydraulic_brakes_f = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t hydraulic_brakes_r = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t pneumatic_brakes_f = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t pneumatic_brakes_r = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t kers               = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t steering           = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t steering_dt        = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t volt               = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t max_cell_volt      = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t power              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t power_dt           = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t vx                 = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t vy                 = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t vz                 = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t ax                 = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t ay                 = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t az                 = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t yaw_dt             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t yaw_dtdt           = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t roll               = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t roll_dt            = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t pitch              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t pitch_dt           = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t trq_fl             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t trq_fr             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t trq_rl             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t trq_rr             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t rpm_fl             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t rpm_fr             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t rpm_rl             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
static const scs_t rpm_rr             = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_DE_CRIT };
/* Outputs */
static const scs_t tv_trq_min_fl = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_min_fr = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_min_rr = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_min_rl = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_max_fl = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_max_fr = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_max_rl = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_trq_max_rr = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_rpm_fl     = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_rpm_fr     = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_rpm_rl     = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
static const scs_t tv_rpm_rr     = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_TV_CRIT };
/* PU setpoints */
static const scs_t pu_trq_min_fl        = { .max = 0.0F, .min = -29.1F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_min_fr        = { .max = 0.0F, .min = -29.1F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_min_rr        = { .max = 0.0F, .min = -29.1F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_min_rl        = { .max = 0.0F, .min = -29.1F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_max_fl        = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_max_fr        = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_max_rl        = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_trq_max_rr        = { .max = 29.1F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_rpm_fl            = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_rpm_fr            = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_rpm_rl            = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_rpm_rr            = { .max = 20000.0F, .min = 0.0F, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_steering_setpoint = { .max = M_PI / 4, .min = -M_PI / 4, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_fx_tv             = { .max = 5000, .min = -5000, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_mz_tv             = { .max = 2000, .min = -2000, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
static const scs_t pu_launch_trigger    = { .max = 2, .min = -1, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_AS_CRIT };
/* SBS SCS Signals */
static const scs_t sbs_apps1              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_CRIT };
static const scs_t sbs_apps2              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_CRIT };
static const scs_t sbs_hydraulic_brakes_f = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_CRIT };
static const scs_t sbs_hydraulic_brakes_r = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_CRIT };
static const scs_t sbs_pneumatic_brakes_f = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_CRIT };
static const scs_t sbs_pneumatic_brakes_r = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_CRIT };
/* SBS SCS TV Signals */
static const scs_t sbs_kers              = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_TV_CRIT };
static const scs_t sbs_steering          = { .max = INF, .min = -INF, .time_out = 100, .time_cnt = 0, .state = 0x00, .type = SCS_SBS_TV_CRIT };

void scs_init(scs_t * scs) {
    /* Inputs */
    scs[SCS_APPS1]              = apps1;
    scs[SCS_APPS2]              = apps2;
    scs[SCS_HYDRAULIC_BRAKES_F] = hydraulic_brakes_f;
    scs[SCS_HYDRAULIC_BRAKES_R] = hydraulic_brakes_r;
    scs[SCS_PNEUMATIC_BRAKES_F] = pneumatic_brakes_f;
    scs[SCS_PNEUMATIC_BRAKES_R] = pneumatic_brakes_r;
    scs[SCS_KERS]               = kers;
    scs[SCS_STEERING]           = steering;
    scs[SCS_STEERING_DT]        = steering_dt;
    scs[SCS_VOLT]               = volt;
    scs[SCS_MAX_CELL_VOLT]      = max_cell_volt;
    scs[SCS_POWER]              = power;
    scs[SCS_POWER_DT]           = power_dt;
    scs[SCS_VX]                 = vx;
    scs[SCS_VY]                 = vy;
    scs[SCS_VZ]                 = vz;
    scs[SCS_AX]                 = ax;
    scs[SCS_AY]                 = ay;
    scs[SCS_AZ]                 = az;
    scs[SCS_YAW_DT]             = yaw_dt;
    scs[SCS_YAW_DTDT]           = yaw_dtdt;
    scs[SCS_ROLL]               = roll;
    scs[SCS_ROLL_DT]            = roll_dt;
    scs[SCS_PITCH]              = pitch;
    scs[SCS_PITCH_DT]           = pitch_dt;
    scs[SCS_TRQ_FL]             = trq_fl;
    scs[SCS_TRQ_FR]             = trq_fr;
    scs[SCS_TRQ_RL]             = trq_rl;
    scs[SCS_TRQ_RR]             = trq_rr;
    scs[SCS_RPM_FL]             = rpm_fl;
    scs[SCS_RPM_FR]             = rpm_fr;
    scs[SCS_RPM_RL]             = rpm_rl;
    scs[SCS_RPM_RR]             = rpm_rr;
    /* Outputs */
    scs[SCS_TV_TRQ_MIN_FL] = tv_trq_min_fl;
    scs[SCS_TV_TRQ_MIN_FR] = tv_trq_min_fr;
    scs[SCS_TV_TRQ_MIN_RL] = tv_trq_min_rl;
    scs[SCS_TV_TRQ_MIN_RR] = tv_trq_min_rr;
    scs[SCS_TV_TRQ_MAX_FL] = tv_trq_max_fl;
    scs[SCS_TV_TRQ_MAX_FR] = tv_trq_max_fr;
    scs[SCS_TV_TRQ_MAX_RL] = tv_trq_max_rl;
    scs[SCS_TV_TRQ_MAX_RR] = tv_trq_max_rr;
    scs[SCS_TV_RPM_FL]     = tv_rpm_fl;
    scs[SCS_TV_RPM_FR]     = tv_rpm_fr;
    scs[SCS_TV_RPM_RL]     = tv_rpm_rl;
    scs[SCS_TV_RPM_RR]     = tv_rpm_rr;
    /* PU setpoint */
    scs[SCS_PU_TRQ_MIN_FL]        = pu_trq_min_fl;
    scs[SCS_PU_TRQ_MIN_FR]        = pu_trq_min_fr;
    scs[SCS_PU_TRQ_MIN_RL]        = pu_trq_min_rl;
    scs[SCS_PU_TRQ_MIN_RR]        = pu_trq_min_rr;
    scs[SCS_PU_TRQ_MAX_FL]        = pu_trq_max_fl;
    scs[SCS_PU_TRQ_MAX_FR]        = pu_trq_max_fr;
    scs[SCS_PU_TRQ_MAX_RL]        = pu_trq_max_rl;
    scs[SCS_PU_TRQ_MAX_RR]        = pu_trq_max_rr;
    scs[SCS_PU_RPM_FL]            = pu_rpm_fl;
    scs[SCS_PU_RPM_FR]            = pu_rpm_fr;
    scs[SCS_PU_RPM_RL]            = pu_rpm_rl;
    scs[SCS_PU_RPM_RR]            = pu_rpm_rr;
    scs[SCS_PU_STEERING_SETPOINT] = pu_steering_setpoint;
    scs[SCS_PU_FX_SETPOINT]       = pu_fx_tv;
    scs[SCS_PU_MZ_SETPOINT]       = pu_mz_tv;
    scs[SCS_PU_LAUNCH_SETPOINT]   = pu_launch_trigger;
    /* SBS SCS Signals */
    scs[SCS_SBS_APPS1]              = sbs_apps1;
    scs[SCS_SBS_APPS2]              = sbs_apps2;
    scs[SCS_SBS_HYDRAULIC_BRAKES_F] = sbs_hydraulic_brakes_f;
    scs[SCS_SBS_HYDRAULIC_BRAKES_R] = sbs_hydraulic_brakes_r;
    scs[SCS_SBS_PNEUMATIC_BRAKES_F] = sbs_pneumatic_brakes_f;
    scs[SCS_SBS_PNEUMATIC_BRAKES_R] = sbs_pneumatic_brakes_r;
    /* SBS SCS TV Signals */
    scs[SCS_SBS_KERS]              = sbs_kers;
    scs[SCS_SBS_STEERING]          = sbs_steering;
}

scs_status_t scs_validate_tv(vcu_data_t * vcu_data_p) {
    scs_status_t status = SCS_OK;

    scs_validate(vcu_data_p->tv_output.inv_sp.trq_max_fl, &(vcu_data_p->scs[SCS_TV_TRQ_MAX_FL]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.trq_max_fr, &(vcu_data_p->scs[SCS_TV_TRQ_MAX_FR]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.trq_max_rl, &(vcu_data_p->scs[SCS_TV_TRQ_MAX_RL]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.trq_max_rr, &(vcu_data_p->scs[SCS_TV_TRQ_MAX_RR]), &status);

    scs_validate(vcu_data_p->tv_output.inv_sp.trq_min_fl, &(vcu_data_p->scs[SCS_TV_TRQ_MIN_FL]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.trq_min_fr, &(vcu_data_p->scs[SCS_TV_TRQ_MIN_FR]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.trq_min_rl, &(vcu_data_p->scs[SCS_TV_TRQ_MIN_RL]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.trq_min_rr, &(vcu_data_p->scs[SCS_TV_TRQ_MIN_RR]), &status);

    scs_validate(vcu_data_p->tv_output.inv_sp.rpm_req_fl, &(vcu_data_p->scs[SCS_TV_RPM_FL]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.rpm_req_fr, &(vcu_data_p->scs[SCS_TV_RPM_FR]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.rpm_req_rl, &(vcu_data_p->scs[SCS_TV_RPM_RL]), &status);
    scs_validate(vcu_data_p->tv_output.inv_sp.rpm_req_rr, &(vcu_data_p->scs[SCS_TV_RPM_RR]), &status);
    return status;
}

scs_status_t scs_validate_pu_setpoints(vcu_data_t * vcu_data_p) {
    scs_status_t status = SCS_OK;

    scs_validate(vcu_data_p->pu_setpoints.trq_max_fl, &(vcu_data_p->scs[SCS_PU_TRQ_MAX_FL]), &status);
    scs_validate(vcu_data_p->pu_setpoints.trq_max_fr, &(vcu_data_p->scs[SCS_PU_TRQ_MAX_FR]), &status);
    scs_validate(vcu_data_p->pu_setpoints.trq_max_rl, &(vcu_data_p->scs[SCS_PU_TRQ_MAX_RL]), &status);
    scs_validate(vcu_data_p->pu_setpoints.trq_max_rr, &(vcu_data_p->scs[SCS_PU_TRQ_MAX_RR]), &status);

    scs_validate(vcu_data_p->pu_setpoints.trq_min_fl, &(vcu_data_p->scs[SCS_PU_TRQ_MIN_FL]), &status);
    scs_validate(vcu_data_p->pu_setpoints.trq_min_fr, &(vcu_data_p->scs[SCS_PU_TRQ_MIN_FR]), &status);
    scs_validate(vcu_data_p->pu_setpoints.trq_min_rl, &(vcu_data_p->scs[SCS_PU_TRQ_MIN_RL]), &status);
    scs_validate(vcu_data_p->pu_setpoints.trq_min_rr, &(vcu_data_p->scs[SCS_PU_TRQ_MIN_RR]), &status);

    scs_validate(vcu_data_p->pu_setpoints.rpm_req_fl, &(vcu_data_p->scs[SCS_PU_RPM_FL]), &status);
    scs_validate(vcu_data_p->pu_setpoints.rpm_req_fr, &(vcu_data_p->scs[SCS_PU_RPM_FR]), &status);
    scs_validate(vcu_data_p->pu_setpoints.rpm_req_rl, &(vcu_data_p->scs[SCS_PU_RPM_RL]), &status);
    scs_validate(vcu_data_p->pu_setpoints.rpm_req_rr, &(vcu_data_p->scs[SCS_PU_RPM_RR]), &status);

    return status;
}

scs_status_t scs_watchdog(scs_t * scs) {
    scs_status_t status = SCS_OK;
    for (u8 i = 0; i < SCS_NUM; i++) {
        if (++(scs[i].time_cnt) >= scs[i].time_out) {
            scs[i].time_cnt = 0;
            scs[i].state |= SCS_TIME_OUT;
            status = scs[i].type > status ? scs[i].type : status;
        }
    }
    return status;
}

f32 scs_validate(f32 val, scs_t * scs, scs_status_t * status) {
    scs->time_cnt = 0;
    scs->state &= ~SCS_TIME_OUT;

    if (val < scs->min) { /* Value is below legal limits */
        scs->state |= SCS_UNDR_SAT;
        *status = scs->type > *status ? scs->type : *status;
        return 0.0;
    } else if (scs->max < val) { /* Value is above legal limits */
        scs->state |= SCS_OVER_SAT;
        *status = scs->type > *status ? scs->type : *status;
        return 0.0;
    } else if (val == val) { /* Value is OK */
        scs->state &= ~(SCS_OVER_SAT | SCS_UNDR_SAT | SCS_NAN);
        return val;
    } else { /* Value is NaN */
        scs->state = SCS_NAN;
        *status    = scs->type > *status ? scs->type : *status;
        return 0.0;
    }
}
