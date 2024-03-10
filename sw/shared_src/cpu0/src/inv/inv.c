//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/inv/inv.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 04-06-19
#include "../inv/inv.h"
#include "../safety/safety.h"
#include "../hsm/hsm.h"

#include "inverter/State_1_0.h"
#include "inverter/Setpoints_1_0.h"

static void inv_ctrl_set_torque_w_state(vcu_data_t * vcu_data_p, f32 trq, u8 inv_inst, u8 same_axis_inv, u8 active_inverters);

static bool is_as_mission(u8 mission);
static bool is_as_inspection_mission(u8 mission);
static bool is_pu_max_torque_equal(vcu_data_t * vcu_data_p);
static bool is_pu_min_torque_equal(vcu_data_t * vcu_data_p);
static bool is_pu_rpm_equal(vcu_data_t * vcu_data_p);
static bool is_pu_setpoints_below_power_limit(u8 mission, u16 rpm, u16 torque);
static bool is_pu_setpoints_below_kers_limit(u8 mission, u16 rpm, u16 torque);

static bool all_equal(u16 a, u16 b, u16 c, u16 d);

void inv_ctrl_static(vcu_data_t * vcu_data_p) {
    u8 current_mission = vcu_data_p->config.mission;
    if (vcu_State_1_0_MANUAL == current_mission) {
        f32 trq = vcu_data_p->tv_input.throttle * vcu_data_p->config.st_trq;
        for (u8 i = 0; i < INVERTER_COUNT; ++i) {
            vcu_data_p->inv_sp[i].trq_max = (u16)(100.0 * trq);
            vcu_data_p->inv_sp[i].trq_min = 0U;
            vcu_data_p->inv_sp[i].rpm_req = (u16)(vcu_data_p->config.st_rpm);
            vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_SETPOINTS : INV_CMD_SETPOINT;
        }
    } else if (is_as_mission(current_mission) == true) {
        u16 trq_max = 100.0 * vcu_data_p->pu_setpoints.trq_max_fl;
        u16 trq_min = -100.0 * vcu_data_p->pu_setpoints.trq_min_fl;
        u16 rpm     = vcu_data_p->pu_setpoints.rpm_req_fl;

        // clang-format off
        if (is_pu_min_torque_equal(vcu_data_p) &&
            is_pu_max_torque_equal(vcu_data_p) &&
            is_pu_rpm_equal(vcu_data_p) &&
            is_pu_setpoints_below_power_limit(current_mission, rpm, trq_max / 100) &&
            is_pu_setpoints_below_kers_limit(current_mission, rpm, trq_min / 100)) {
            // clang-format on
            for (u8 i = 0; i < INVERTER_COUNT; ++i) {
                vcu_data_p->inv_sp[i].trq_max = trq_max;
                vcu_data_p->inv_sp[i].trq_min = trq_min;
                vcu_data_p->inv_sp[i].rpm_req = rpm;
                vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_SETPOINTS : INV_CMD_SETPOINT;
            }
        } else {
            hsm_add_ev(EV_ERROR_INVERTER_SETPOINTS_PU);
        }
    }
}

void inv_ctrl_static_speed(vcu_data_t * vcu_data_p) {
    f32 rpm = vcu_data_p->tv_input.throttle * vcu_data_p->config.st_rpm;

    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].trq_max = (u16)(100.0 * vcu_data_p->config.st_trq);
        vcu_data_p->inv_sp[i].trq_min = 0U;
        vcu_data_p->inv_sp[i].rpm_req = (u16)rpm;
        vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_SETPOINTS : INV_CMD_SETPOINT;
    }
}

void inv_ctrl_static_limp(vcu_data_t * vcu_data_p) {
    f32 trq = vcu_data_p->tv_input.throttle * vcu_data_p->config.st_trq;

    // Find number of active inverters
    u8 active_inverters = 0;
    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        if (vcu_data_p->inverter_type == INV_TYPE_I21) {
            if (vcu_data_p->inv_state[i] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[i] == inverter_State_1_0_STATE_ENABLED) {
                active_inverters++;
            }
        } else {
            if (vcu_data_p->inv_state[i] == INV_STATE_READY) {
                active_inverters++;
            }
        }
    }

    inv_ctrl_set_torque_w_state(vcu_data_p, trq, FL, FR, active_inverters);
    inv_ctrl_set_torque_w_state(vcu_data_p, trq, FR, FL, active_inverters);
    inv_ctrl_set_torque_w_state(vcu_data_p, trq, RL, RR, active_inverters);
    inv_ctrl_set_torque_w_state(vcu_data_p, trq, RR, RL, active_inverters);

    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].trq_min = 0U;
        vcu_data_p->inv_sp[i].rpm_req = (u16)(vcu_data_p->config.st_rpm);
    }
}

void inv_ctrl_zero(vcu_data_t * vcu_data_p) {
    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].trq_max = 0U;
        vcu_data_p->inv_sp[i].trq_min = 0U;
        vcu_data_p->inv_sp[i].rpm_req = 0U;
        vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_SETPOINTS : INV_CMD_SETPOINT;
    }
}

void inv_ctrl_tv(vcu_data_t * vcu_data_p) {
    vcu_data_p->inv_sp[FL].trq_max = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_max_fl));
    vcu_data_p->inv_sp[FR].trq_max = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_max_fr));
    vcu_data_p->inv_sp[RL].trq_max = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_max_rl));
    vcu_data_p->inv_sp[RR].trq_max = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_max_rr));

    vcu_data_p->inv_sp[FL].trq_min = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_min_fl));
    vcu_data_p->inv_sp[FR].trq_min = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_min_fr));
    vcu_data_p->inv_sp[RL].trq_min = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_min_rl));
    vcu_data_p->inv_sp[RR].trq_min = (u16)(100.0 * (vcu_data_p->tv_output.inv_sp.trq_min_rr));

    vcu_data_p->inv_sp[FL].rpm_req = (u16)(vcu_data_p->tv_output.inv_sp.rpm_req_fl);
    vcu_data_p->inv_sp[FR].rpm_req = (u16)(vcu_data_p->tv_output.inv_sp.rpm_req_fr);
    vcu_data_p->inv_sp[RL].rpm_req = (u16)(vcu_data_p->tv_output.inv_sp.rpm_req_rl);
    vcu_data_p->inv_sp[RR].rpm_req = (u16)(vcu_data_p->tv_output.inv_sp.rpm_req_rr);

    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].cmd = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_SETPOINTS : INV_CMD_SETPOINT;
    }
}

void inv_ctrl_enable_fw(vcu_data_t * vcu_data_p) {
    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].trq_max = 0U;
        vcu_data_p->inv_sp[i].trq_min = 0U;
        vcu_data_p->inv_sp[i].rpm_req = 0U;
        vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_ENABLE_W_FW :  INV_CMD_ENABLE | INV_CMD_FW;
    }
}

void inv_ctrl_enable(vcu_data_t * vcu_data_p) {
    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].trq_max = 0U;
        vcu_data_p->inv_sp[i].trq_min = 0U;
        vcu_data_p->inv_sp[i].rpm_req = 0U;
        vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_ENABLE_WO_FW : INV_CMD_ENABLE;
    }
}

void inv_ctrl_disable(vcu_data_t * vcu_data_p) {
    for (u8 i = 0; i < INVERTER_COUNT; ++i) {
        vcu_data_p->inv_sp[i].trq_max = 0U;
        vcu_data_p->inv_sp[i].trq_min = 0U;
        vcu_data_p->inv_sp[i].rpm_req = 0U;
        vcu_data_p->inv_sp[i].cmd     = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_DISABLE : INV_CMD_DISABLE;
    }
}

/**
 * @brief Sets requested torque to zero or trq based on inv_inst inverters state and same axis inverters. If there is two inverters left it will use all available. Otherwise it will only use one axis
 *
 * @param vcu_data_p pointer to vcu_data_t with inverter state and torque messages
 * @param trq The requested torque
 * @param inv_inst inverter instance to request torque from
 * @param same_axis_inv inverter on the same axis, f.eks will FR's same axis be FL
 * @param active_inverters The number of active inverters
 */
static void inv_ctrl_set_torque_w_state(vcu_data_t * vcu_data_p, f32 trq, u8 inv_inst, u8 same_axis_inv, u8 active_inverters) {
    if (vcu_data_p->config.mission != vcu_State_1_0_MANUAL) {
        return;
    }

    bool ready;
    bool other_ready;

    if (vcu_data_p->inverter_type == INV_TYPE_I21) {
        ready       = inverter_State_1_0_STATE_ARMED == vcu_data_p->inv_state[inv_inst] || inverter_State_1_0_STATE_ENABLED == vcu_data_p->inv_state[inv_inst];
        other_ready = inverter_State_1_0_STATE_ARMED == vcu_data_p->inv_state[same_axis_inv] || inverter_State_1_0_STATE_ENABLED == vcu_data_p->inv_state[same_axis_inv];
    } else {
        ready       = vcu_data_p->inv_state[inv_inst] == INV_STATE_READY;
        other_ready = vcu_data_p->inv_state[same_axis_inv] == INV_STATE_READY;
    }


    if (ready && (other_ready || active_inverters <= 2)) {
        vcu_data_p->inv_sp[inv_inst].trq_max = (u16)(100.0 * trq);
    } else {
        vcu_data_p->inv_sp[inv_inst].trq_max = 0U;
    }

    vcu_data_p->inv_sp[inv_inst].cmd = (vcu_data_p->inverter_type == INV_TYPE_I21) ? inverter_Setpoint_1_0_COMMAND_SETPOINTS : INV_CMD_SETPOINT;
}

static bool is_as_mission(u8 mission) {
    switch (mission) {
    case vcu_State_1_0_AUTONOMOUS_ACCELERATION:
    case vcu_State_1_0_AUTONOMOUS_SKID:
    case vcu_State_1_0_AUTONOMOUS_TRACKDRIVE:
    case vcu_State_1_0_EBS_TEST:
    case vcu_State_1_0_AUTONOMOUS_AUTOCROSS:
    case vcu_State_1_0_INSPECTION:
        return true;
    default:
        return false;
    }
}

static bool is_as_inspection_mission(u8 mission) {
    return mission == vcu_State_1_0_INSPECTION;
}

static bool is_pu_max_torque_equal(vcu_data_t * vcu_data_p) {
    return all_equal(vcu_data_p->pu_setpoints.trq_max_fl,
                     vcu_data_p->pu_setpoints.trq_max_fr,
                     vcu_data_p->pu_setpoints.trq_max_rl,
                     vcu_data_p->pu_setpoints.trq_max_rr);
}
static bool is_pu_min_torque_equal(vcu_data_t * vcu_data_p) {
    return all_equal(vcu_data_p->pu_setpoints.trq_min_fl,
                     vcu_data_p->pu_setpoints.trq_min_fr,
                     vcu_data_p->pu_setpoints.trq_min_rl,
                     vcu_data_p->pu_setpoints.trq_min_rr);
}

static bool is_pu_rpm_equal(vcu_data_t * vcu_data_p) {
    return all_equal(vcu_data_p->pu_setpoints.rpm_req_fl,
                     vcu_data_p->pu_setpoints.rpm_req_fr,
                     vcu_data_p->pu_setpoints.rpm_req_rl,
                     vcu_data_p->pu_setpoints.rpm_req_rr);
}

static bool is_pu_setpoints_below_power_limit(u8 mission, u16 rpm, u16 torque) {
    if (is_as_inspection_mission(mission)) {
        return safety_check_as_inspection_mission_power_limit(rpm, torque) == SAFE;
    } else if (is_as_mission(mission)) {
        return safety_check_as_power_limit(rpm, torque) == SAFE;
    } else {
        return false;
    }
}

static bool is_pu_setpoints_below_kers_limit(u8 mission, u16 rpm, u16 torque) {
    if (is_as_inspection_mission(mission)) {
        return safety_check_as_inspection_mission_kers_limit(rpm, torque) == SAFE;
    } else if (is_as_mission(mission)) {
        return safety_check_as_kers_limit(rpm, torque) == SAFE;
    } else {
        return false;
    }
}

static bool all_equal(u16 a, u16 b, u16 c, u16 d) {
    return (a == b) && (b == c) && (c == d);
}
