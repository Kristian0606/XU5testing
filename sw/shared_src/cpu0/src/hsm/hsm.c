//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/hsm/hsm.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 24-04-19

#include "hsm.h"
#include "../remote_emergency_system/remote_emergency_system.h"
#include "../telemetry/telemetry.h"
#include "../actuators/actuators.h"
#include "../timer/ttc_timer.h"
#include "../safety/safety.h"
#include "../utility/log.h"
#include "../shared_src/utility/mem.h"
#include "../utility/pmu.h"
#include "../epos/epos.h"
#include "../safety/scs.h"
#include "../gpio/gpio.h"
#include "../zcan/zcan.h"
#include "../shared_src/intc/intc.h"
#include "../assi/assi.h"
#include "../sdc/sdc.h"
#include "../inv/inv.h"
#include "../ins/ins.h"
#include "../shared_src/qu/qu.h"
#include "common/Systems_1_0.h"
#include "ams/State_1_0.h"
#include "inverter/State_1_0.h"
#include "../shared_src/ipi/ipi.h"

// constants
const u32 INVERTER_TIMEOUT_MSEC     = 2;    // how often the inverter is checked
const f32 AS_FINISHED_RPM_THRESHOLD = 50.0; // rpm threshold for transitioning to as_finished

typedef struct hsm_t hsm_t;
typedef void * (*state_fp)(hsm_t * hsm_p, u8 ev);
struct hsm_t {
    state_fp state;
};

static volatile qu_t ev_qu = { .head = 0, .tail = 0 };
static vcu_data_t *  vcu_data_p;
static XIpiPsu          ipi_i;

static void hsm_init(hsm_t * hsm_p, state_fp init_state);
static void hsm_dispatch(hsm_t * hsm_p, u8 ev);
static void hsm_transition(hsm_t * hsm_p, state_fp new_state);
static void hsm_transition_to_idle_or_emergency(hsm_t * hsm_p);
static void leave_de(hsm_t * hsm_p, u8 warning, u8 reason);
static void clear_pu_setpoints();

static void * state_mission_select(hsm_t * hsm_p, u8 ev);
static void * state_idle_autonomous(hsm_t * hsm_p, u8 ev);
static void * state_idle_manual(hsm_t * hsm_p, u8 ev);
static void * state_rtds_request(hsm_t * hsm_p, u8 ev);
static void * state_rtds_playing(hsm_t * hsm_p, u8 ev);
static void * state_en_inv(hsm_t * hsm_p, u8 ev);

static void * duper_de(hsm_t * hsm_p, u8 ev);
static void * super_de_manual(hsm_t * hsm_p, u8 ev);
static void * super_de_as(hsm_t * hsm_p, u8 ev);

static void * state_de_selector(hsm_t * hsm_p, u8 ev);
static void * state_de_static(hsm_t * hsm_p, u8 ev);
static void * state_de_rc(hsm_t * hsm_p, u8 ev);
static void * state_de_rc_static(hsm_t * hsm_p, u8 ev);
static void * state_de_lc(hsm_t * hsm_p, u8 ev);
static void * state_de_skid(hsm_t * hsm_p, u8 ev);
static void * state_de_limp(hsm_t * hsm_p, u8 ev);

static void * state_as_ready(hsm_t * hsm_p, u8 ev);
static void * state_as_finished(hsm_t * hsm_p, u8 ev);
static void * state_as_emergency(hsm_t * hsm_p, u8 ev);

static void * state_de_as_direct(hsm_t * hsm_p, u8 ev);
static void * state_de_as_lc(hsm_t * hsm_p, u8 ev);
static void * state_de_as_rc(hsm_t * hsm_p, u8 ev);

// High level overview over the hsm
// ================================
// The Hierarchical State Machine (dot) c is an implementation of a hierarcical
// state machine
//
// Low level overview of the hsm
// =============================
// Queue
// -----
// Uses a qu.h (queue) instace, where events are added thru hsm_add_ev or
// hsm_add_ev_isr for interrupts.
//
// Certain events are handled directly by the hsm, some are first handled in
// the loop and then sent to the current state (ex. EV_1HZ_TICK), others are
// just sent to the current state directly.
//
// Events can be directly dispatched with hsm_dispatch(), which bypasses the qu
//
// States
// ------
// States are represented as functions. The current state is simply the
// function which gets called with new events.
//
// A state can return another state, sending the event to that superstate
// aswell. Useful with ex. the inverters. We use a super-state in all the drive
// modes, which handles inverter communication.

// Enqueues events based on status, if there is a failure
void hsm_check_scs(scs_status_t status) {
#ifdef PU_TEST
    return;
#endif
#ifdef TV_HIL_TEST
    return;
#endif

    switch (status) {
    case SCS_DE_CRIT:
        hsm_add_ev(EV_SCS_DE_FAILURE);
        break;
    case SCS_SBS_CRIT:
        hsm_add_ev(EV_SBS_SCS_DE_FAILURE);
        break;
    case SCS_TV_CRIT:
        hsm_add_ev(EV_SCS_TV_FAILURE);
        hsm_add_ev(EV_AS_FAILURE);
        break;
    case SCS_SBS_TV_CRIT:
        hsm_add_ev(EV_SBS_SCS_TV_FAILURE);
        hsm_add_ev(EV_AS_FAILURE);
        break;
    case SCS_AS_CRIT:
        hsm_add_ev(EV_AS_FAILURE);
        break;
    default:
        // no failure :)
        break;
    }
}

void hsm_run(vcu_data_t * data_p, XIpiPsu * ipi_p) {
    vcu_data_p = data_p;
    ipi_i = *ipi_p;
    hsm_t hsm_i;
    u8    in_ev;
    bool  asms_previous_status = false;
    /* Send initial data to satisfy Analyze plug-ins */
    zcan_tx_implausibilities();
    zcan_tx_config_to_default_node();


    handle_assi(AS_off);
    scs_init(&vcu_data_p->scs[0]);
    hsm_init(&hsm_i, state_mission_select);

#ifdef TV_HIL_TEST
    vcu_data_p->config.mission = vcu_State_1_0_MANUAL;
    hsm_add_ev(EV_DEBUG_DE);
#endif

    while (1) {
        dequ(&ev_qu, &in_ev);
        vcu_data_p->status.cpu0.ev_hz++;
        /*
         * Some events are handled independently of state,
         * while others are dispatched to the current state
         * in the default condition.
         */
        switch (in_ev) {
        case EV_SYNC_STATUS:
            mem_r_cpu1_status(&vcu_data_p->status.cpu1);
            break;
        case EV_INS_RX_MSG:
            ins_get_msg(vcu_data_p);
            zcan_tx_ins();
            zcan_tx_gnss();
            zcan_tx_telemetry_ins_estimates_1();
            zcan_tx_telemetry_ins_estimates_2();
            zcan_tx_imu_measurements();
            zcan_tx_ins_status();
            break;
        case EV_RX_CANFD0: // these all fall through
        case EV_RX_CANFD1: // these all fall through
        case EV_RX_CANFD2: // to here
            zcan_canfd_rx(in_ev);
            break;
        case EV_RX_CAN_OPEN0: // these all fall through
        case EV_RX_CAN_OPEN1: // these all fall through
        case EV_RX_CAN_OPEN2: // to here
            //zcan_can_open_rx(in_ev); TODO: Uncomment when canopen is implemented
            break;
        case EV_TV_CONFIG_START:
            vcu_data_p->status.tv_cfg_busy = SET;
            mem_w_tv_config(&vcu_data_p->tv_config);
            ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_CONFIGURE);
            break;
        case EV_TV_CONFIG_DONE:
            mem_r_tv_config(&vcu_data_p->tv_config);
            zcan_tx_tv_config(&vcu_data_p->tv_config, vcu_data_p->tv_config_destination_id);
            zcan_tx_tv_config(&vcu_data_p->tv_config, common_Systems_1_0_ID_DASHBOARD);
            vcu_data_p->status.tv_cfg_busy = CLEAR;
            break;
        case EV_DEBUG_DE:
            hsm_transition(&hsm_i, state_de_selector);
            break;
        case EV_10KILOHZ_TICK:
            telemetry_poll_ethernet();
            break;
        case EV_100HZ_TICK:
            sdc_toggle_watchdog();
            hsm_dispatch(&hsm_i, in_ev);
            break;
        case EV_20HZ_TICK:
            zcan_tx_telemetry_dv_state();
            break;
        case EV_1HZ_TICK:
            zcan_tx_vcu_status();
            toggle_gpio_pin(PS_DEBUG);
            vcu_data_p->status.cpu0.load   = pmu_get_utilization();
            //vcu_data_p->status.cpu0.ev_buf = qulvl(&ev_qu);
            //ipi_send(&ipi_i, IPI_APU1_ID, 55); // For testing purposes
            mem_r_cpu1_status(&vcu_data_p->status.cpu1); // For testing purposes
            zcan_tx_status();
            zcan_tx_state();
            u32 asms_status = read_gpio_pin(ASMS_ACTUALLY_ON);
            if (asms_status != asms_previous_status) {
                if (asms_status) {
                    hsm_add_ev(EV_ASMS_ON);
                } else {
                    hsm_add_ev(EV_ASMS_OFF);
                }
                asms_previous_status = asms_status;
            }
            if (!ins_is_inited()) {
                ins_continue_init();
            }
            if (!telemetry_is_up()) {
                telemetry_continue_init();
            }
            if (asms_status) {
                //epos_init(); TODO: COMMENT IN WHEN CAN OPEN IS BACK
            } else {
                epos_deinitialize();
            }
            vcu_data_p->status.ins_rx_cnt = 0;
            vcu_data_p->status.cpu0.ev_hz = 0;
            hsm_dispatch(&hsm_i, in_ev);
            break;
        default:
            hsm_dispatch(&hsm_i, in_ev);
            break;
        }
    }
}

void hsm_add_ev(hsm_events_t ev) {
    enqu(&ev_qu, ev);
}

void hsm_add_ev_isr(hsm_events_t ev) {
    enqu_isr(&ev_qu, ev);
}

static void * state_mission_select(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        vcu_data_p->config.mission    = vcu_State_1_0_NO_MISSION;
        vcu_data_p->status.cpu0.state = vcu_State_1_0_MISSION_SELECT;
        handle_assi(AS_off);
        zcan_tx_set_ebs_failure_led(false);
        zcan_tx_stop_playing_as_emergency();
        zcan_tx_state();
        return NULL;
    case EV_REGISTER_I19:
        vcu_data_p->inverter_type = INV_TYPE_I19;
        return NULL;
    case EV_REGISTER_I21:
        vcu_data_p->inverter_type = INV_TYPE_I21;
        return NULL;
    case EV_REQ_NO_MISSION:
        vcu_data_p->config.mission    = vcu_State_1_0_NO_MISSION;
        vcu_data_p->status.cpu0.state = vcu_State_1_0_MISSION_SELECT;
        zcan_tx_state();
        return NULL;
    case EV_REQ_AUTONOMOUS_ACCELERATION:
        vcu_data_p->config.mission = vcu_State_1_0_AUTONOMOUS_ACCELERATION;
        hsm_transition(hsm_p, state_idle_autonomous);
        return NULL;
    case EV_REQ_AUTONOMOUS_SKID:
        vcu_data_p->config.mission = vcu_State_1_0_AUTONOMOUS_SKID;
        hsm_transition(hsm_p, state_idle_autonomous);
        return NULL;
    case EV_REQ_AUTONOMOUS_TRACKDRIVE:
        vcu_data_p->config.mission = vcu_State_1_0_AUTONOMOUS_TRACKDRIVE;
        hsm_transition(hsm_p, state_idle_autonomous);
        return NULL;
    case EV_REQ_EBS_TEST:
        vcu_data_p->config.mission = vcu_State_1_0_EBS_TEST;
        hsm_transition(hsm_p, state_idle_autonomous);
        return NULL;
    case EV_REQ_INSPECTION:
        vcu_data_p->config.mission = vcu_State_1_0_INSPECTION;
        hsm_transition(hsm_p, state_idle_autonomous);
        return NULL;
    case EV_REQ_AUTONOMOUS_AUTOCROSS:
        vcu_data_p->config.mission = vcu_State_1_0_AUTONOMOUS_AUTOCROSS;
        hsm_transition(hsm_p, state_idle_autonomous);
        return NULL;
    case EV_REQ_MANUAL:
        vcu_data_p->config.mission = vcu_State_1_0_MANUAL;
        hsm_transition(hsm_p, state_idle_manual);
        return NULL;
    case EV_UPDATE_ASSI:
        handle_assi(AS_off);
        return NULL;
    }
}

static void * state_idle_autonomous(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_IDLE_AUTONOMOUS;
        vcu_data_p->config.mode = vcu_State_1_0_STATIC;
        handle_assi(AS_off);
        actuators_activate_ebs();
        zcan_tx_state();
#ifdef PU_TEST
        hsm_add_ev(EV_INITIAL_CHECKUP_SEQUENCE_SUCCESS);
#else
        safety_reset_initial_checkup_sequence();
#endif
        return NULL;
    case EV_REQ_NO_MISSION:
        hsm_transition(hsm_p, state_mission_select);
        return NULL;
    case EV_ASMS_ON:
        safety_start_initial_checkup_sequence(vcu_data_p);
        return NULL;
    case EV_ASMS_OFF:
        safety_reset_initial_checkup_sequence();
        return NULL;
    case EV_TSAB_EXTERNAL_BUTTON:
        safety_start_initial_checkup_sequence(vcu_data_p);
        return NULL;
    case EV_INITIAL_CHECKUP_SEQUENCE_SUCCESS:
        hsm_transition(hsm_p, state_as_ready);
        return NULL;
    case EV_INITIAL_CHECKUP_SEQUENCE_FAILED:
        safety_reset_initial_checkup_sequence();
        return NULL;
    case EV_TSAB_COCKPIT_BUTTON:
        zcan_tx_warning(vcu_Warning_1_0_TSAB_DENIED, vcu_Warning_1_0_TA_DEG_SAMMEN);
        return NULL;
    }
}

static void * state_idle_manual(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_IDLE_MANUAL;
        handle_assi(AS_off);
        zcan_tx_state();
        sdc_enable_watchdog();
        sdc_activate_ts();
        return NULL;
    case EV_REQ_NO_MISSION:
        hsm_transition(hsm_p, state_mission_select);
        return NULL;
    case EV_TSAB_EXTERNAL_BUTTON:
        zcan_tx_warning(vcu_Warning_1_0_TSAB_DENIED, vcu_Warning_1_0_TA_DEG_SAMMEN);
        return NULL;
    case EV_TSAB_COCKPIT_BUTTON:
        zcan_tx_activate_ts();
        return NULL;
    case EV_REQ_ENTER_DE:
        if (vcu_data_p->ams_state == ams_State_1_0_STATE_TS_ACTIVE) {
            if (safety_check_brake_pressure(vcu_data_p) == SAFE) {
                hsm_transition(hsm_p, state_rtds_request);
                zcan_tx_drivemode_response(GRANTED);
            } else {
                zcan_tx_warning(vcu_Warning_1_0_DE_DENIED, vcu_Warning_1_0_HYDRAULIC_ENGAGED_TOO_LOW);
                zcan_tx_drivemode_response(DENIED);
            }
        } else {
            zcan_tx_warning(vcu_Warning_1_0_DE_DENIED, vcu_Warning_1_0_TS_NOT_ACTIVE);
            zcan_tx_drivemode_response(DENIED);
        }
        return NULL;
    }
}

static void * state_as_ready(hsm_t * hsm_p, u8 ev) {
    static bool waiting_in_as_ready = true;
    static int  one_hz_tick_counter = 0;

    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_AS_READY;
        one_hz_tick_counter           = 0;
        waiting_in_as_ready           = true;
        clear_pu_setpoints();
        zcan_tx_state();
        return NULL;
    case EV_START_CONTINUOUS_MONITORING:
#ifdef PU_TEST
        // skip monitoring when testing
#else
        safety_continuous_monitoring(vcu_data_p, false);
#endif
        return NULL;
    case EV_CONTINUOUS_MONITORING_FAILED:
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
    case EV_AS_FAILURE:
        zcan_tx_warning(vcu_Warning_1_0_AS_DRIVING_DENIED, vcu_Warning_1_0_SCS_FAILURE);
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
    case EV_RES_GO_PRESSED:
        if (waiting_in_as_ready) {
            zcan_tx_warning(vcu_Warning_1_0_AS_DRIVING_DENIED, vcu_Warning_1_0_WAIT_IN_AS_READY);
            zcan_tx_drivemode_response(DENIED);
        } else {
            actuators_deactivate_ebs();
            hsm_transition(hsm_p, state_rtds_request);
            zcan_tx_drivemode_response(GRANTED);
        }
        return NULL;
    case EV_1HZ_TICK:
        ++one_hz_tick_counter;
        if (one_hz_tick_counter == 6) { // Minimum 5 seconds according to the rules.
            waiting_in_as_ready = false;
#ifdef PU_TEST
            // spoof go pressed
            hsm_add_ev(EV_RES_GO_PRESSED);
#endif
        }
        return NULL;
    case EV_UPDATE_ASSI:
        handle_assi(AS_ready);
        return NULL;
    case EV_TS_OFF:
        hsm_transition_to_idle_or_emergency(hsm_p);
        return NULL;
    }
}

static void * state_as_finished(hsm_t * hsm_p, u8 ev) {
#ifdef PU_TEST
    static int one_hz_tick_counter = 0;
#endif
    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_AS_FINISHED;
        actuators_activate_ebs();
        sdc_trigger_latch();
        sdc_deactivate_ts();
        zcan_tx_deactivate_ts();
        inv_ctrl_disable(vcu_data_p);
        zcan_tx_inv_ctrl();
        handle_assi(AS_finished);
        zcan_tx_state();
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_IDLE);
        return NULL;
    case EV_ASMS_OFF:
        if (!actuators_is_hydraulic_brakes_engaged(vcu_data_p)) {
            hsm_transition(hsm_p, state_mission_select);
        }
        return NULL;
    case EV_5HZ_TICK:
        if (true == sdc_is_ts_on() || ams_State_1_0_STATE_TS_ACTIVE == vcu_data_p->ams_state) {
            zcan_tx_deactivate_ts();
        }
        return NULL;
#ifdef PU_TEST
    case EV_1HZ_TICK:
        one_hz_tick_counter++;
        if (one_hz_tick_counter == 10) {
            one_hz_tick_counter = 0;
            hsm_transition(hsm_p, state_mission_select);
        }
        return NULL;
#endif
    }
}

static void * state_as_emergency(hsm_t * hsm_p, u8 ev) {
    static int five_hz_counter    = 0;
    const int  COUNTER_10_SECONDS = 50;
    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_AS_EMERGENCY;
        actuators_activate_ebs();
        sdc_trigger_latch();
        sdc_deactivate_ts();
        zcan_tx_deactivate_ts();
        inv_ctrl_disable(vcu_data_p);
        zcan_tx_inv_ctrl();
        zcan_tx_start_playing_as_emergency();
        zcan_tx_state();
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_IDLE);
        five_hz_counter = 0;
        return NULL;
    case EV_5HZ_TICK:
        if (true == sdc_is_ts_on() || ams_State_1_0_STATE_TS_ACTIVE == vcu_data_p->ams_state) {
            zcan_tx_deactivate_ts();
        }
        if (five_hz_counter > COUNTER_10_SECONDS) {
            zcan_tx_stop_playing_as_emergency();
            sdc_try_unlatch();
        }
        ++five_hz_counter;
        return NULL;
    case EV_ASMS_OFF:
        if (!actuators_is_hydraulic_brakes_engaged(vcu_data_p)) {
            hsm_transition(hsm_p, state_mission_select);
        }
        return NULL;
    case EV_UPDATE_ASSI:
        handle_assi(AS_emergency);
        return NULL;
    }
}

// RTDS super-state for handling EV_TS_OFF
//
// Rule: EV 4.12.1 The vehicle must make a characteristic sound, continuously
// for at least one second and a maximum of three seconds when it enters
// ready-to-drive mode.
static void * super_rtds(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:       // fallthru
    case EV_ENTRY: // fallthru
    case EV_EXIT:
        return NULL; // no super state
    case EV_TS_OFF:
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_RTDS, vcu_Warning_1_0_TS_NOT_ACTIVE);
        hsm_transition_to_idle_or_emergency(hsm_p);
        return NULL;
    }
}

// Request the dashboard to play ReadyToDriveSound
static void * state_rtds_request(hsm_t * hsm_p, u8 ev) {
    static const u32 req_interval = 1000U; // one second
    static const u8  req_thres    = 5;
    static u8        req_count    = 0;

    switch (ev) {
    default:
        return super_rtds;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_RTDS_REQ;
        zcan_tx_state();
        ttc_timer_schedule_interrupt(req_interval, TTC_TIMER_3);
        zcan_tx_play_rtds();
        req_count = 0;
#ifdef PU_TEST
        hsm_add_ev(EV_RTDS_GRANTED);
#endif
        return NULL;
    case EV_EXIT:
        return NULL;
    case EV_TIMEOUT:
        req_count++;

        if (req_count >= req_thres) {
            zcan_tx_warning(vcu_Warning_1_0_LEAVING_RTDS, vcu_Warning_1_0_TIMED_OUT);
            hsm_transition_to_idle_or_emergency(hsm_p);
        } else {
            ttc_timer_schedule_interrupt(req_interval, TTC_TIMER_3);
            // ask dash to play RTDS
            zcan_tx_play_rtds();
        }
        return NULL;
    case EV_RTDS_DENIED:
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_RTDS, vcu_Warning_1_0_DASH_REQUEST);
        hsm_transition_to_idle_or_emergency(hsm_p);
        return NULL;
    // dash has started playing RTDS
    case EV_RTDS_GRANTED:
        hsm_transition(hsm_p, state_rtds_playing);
        return NULL;
    }
}

// Dash is playing ReadyToDriveSound
static void * state_rtds_playing(hsm_t * hsm_p, u8 ev) {
    static const u32 rtds_timeout = 10000U;
    switch (ev) {
    default:
        return super_rtds;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_RTDS_PLAY;
        zcan_tx_state();
        ttc_timer_schedule_interrupt(rtds_timeout, TTC_TIMER_3);
#ifdef PU_TEST
        hsm_add_ev(EV_RTDS_FINISHED);
#endif
        return NULL;
    case EV_EXIT:
        return NULL;
    case EV_TIMEOUT:
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_RTDS, vcu_Warning_1_0_TIMED_OUT);
        hsm_transition_to_idle_or_emergency(hsm_p);
        return NULL;
    // dash says it's done playing
    case EV_RTDS_FINISHED:
        hsm_transition(hsm_p, state_en_inv);
        return NULL;
    }
}

// Enable Inverter
//
// Once Inverter is *good*, goes to state selector
static void * state_en_inv(hsm_t * hsm_p, u8 ev) {
    static u8 tick_count = 0;
    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY: {
        // start timer for sending setpoints
        axi_timer_start(TIMER_INV, INVERTER_TIMEOUT_MSEC);
        tick_count                    = 0;
        vcu_data_p->status.cpu0.state = vcu_State_1_0_EN_INV;
        zcan_tx_state();
        /* Inverter Enable Sequence       */
        /* 1. Send blank control message  */
        inv_ctrl_zero(vcu_data_p);
        zcan_tx_inv_ctrl();
        /* 2. Send enable control message  */
        inv_ctrl_enable_fw(vcu_data_p);
        zcan_tx_inv_ctrl();
        /* 3. Wait for inverter states to change to ready */

#ifdef PU_TEST
        hsm_transition(hsm_p, state_de_selector);
#endif
        return NULL;
    }
    case EV_EXIT:
        // don't stop the timer, because it's used in other states
        return NULL;
    case EV_INV_READY:
        // clang-format off
        switch (vcu_data_p->inverter_type) {
        	default:
        		break;
            case INV_TYPE_I21:
                if ((vcu_data_p->inv_state[FL] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[FL] == inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[FR] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[FR] == inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[RL] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[RL] == inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[RR] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[RR] == inverter_State_1_0_STATE_ENABLED)
                    ) {
                    hsm_transition(hsm_p, state_de_selector);
                } else if (
                    (vcu_data_p->inv_state[FL] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[FL] == inverter_State_1_0_STATE_ENABLED) ||
                    (vcu_data_p->inv_state[FR] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[FR] == inverter_State_1_0_STATE_ENABLED) ||
                    (vcu_data_p->inv_state[RL] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[RL] == inverter_State_1_0_STATE_ENABLED) ||
                    (vcu_data_p->inv_state[RR] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[RR] == inverter_State_1_0_STATE_ENABLED)
                    ) {
                    // what if we're in AS modes?
                    hsm_transition(hsm_p, state_de_limp);
                }
                break;
            case INV_TYPE_I19:
                if ((vcu_data_p->inv_state[FL] == INV_STATE_READY) &&
                    (vcu_data_p->inv_state[FR] == INV_STATE_READY) &&
                    (vcu_data_p->inv_state[RL] == INV_STATE_READY) &&
                    (vcu_data_p->inv_state[RR] == INV_STATE_READY)
                    ) {
                    hsm_transition(hsm_p, state_de_selector);
                } else if (
                    (vcu_data_p->inv_state[FL] == INV_STATE_READY) ||
                    (vcu_data_p->inv_state[FR] == INV_STATE_READY) ||
                    (vcu_data_p->inv_state[RL] == INV_STATE_READY) ||
                    (vcu_data_p->inv_state[RR] == INV_STATE_READY)
                    ) {
                    // what if we're in AS modes?
                    hsm_transition(hsm_p, state_de_limp);
                }
                break;
        }
        // clang-format on
        return NULL;
    case EV_1HZ_TICK:
        tick_count++;
        if (tick_count >= 10) {
            zcan_tx_warning(vcu_Warning_1_0_DE_DENIED, vcu_Warning_1_0_INV_NOT_READY);
            hsm_transition_to_idle_or_emergency(hsm_p);
        }
        return NULL;
    case EV_INV_TIMEOUT:
        // Sending blank setpoint messages to inverters
        inv_ctrl_zero(vcu_data_p);
        return NULL;
    case EV_TS_OFF:
        zcan_tx_warning(vcu_Warning_1_0_DE_DENIED, vcu_Warning_1_0_TS_NOT_ACTIVE);
        hsm_transition_to_idle_or_emergency(hsm_p);
        return NULL;
    }
}

// super duper state: Drive Enable
// Handles:
// * TV
static void * duper_de(hsm_t * hsm_p, u8 ev) {
    static u8 tv_initialized = false;

    switch (ev) {
    default:
        return NULL;
    case EV_ENTRY:
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_INIT);
        tv_initialized = true;
        return NULL;
    case EV_EXIT:
        return NULL;
    case EV_DRIVEMODE_REQ:
        hsm_transition(hsm_p, state_de_selector);
        return NULL;
    case EV_SYNC_TV_INPUT:
        mem_w_tv_input(&vcu_data_p->tv_input);
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_SYNC_INPUT);
        return NULL;
    case EV_SYNC_TV_OUTPUT:
        mem_r_tv_output(&vcu_data_p->tv_output);
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_SYNC_OUTPUT);
        hsm_dispatch(hsm_p, EV_TX_TV_OUTPUT);
        zcan_tx_tv_status();
        zcan_tx_tv_debug();
        return NULL;
    case EV_INV_DISABLED:
        tv_initialized = false;
        zcan_tx_warning(vcu_Warning_1_0_ENTERING_LIMP, vcu_Warning_1_0_INV_NOT_READY);
        hsm_transition(hsm_p, state_de_limp);
        return NULL;
    case EV_TS_OFF:
        tv_initialized = false;
        leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_TS_NOT_ACTIVE);
        return NULL;
    case EV_REQ_EXIT_DE:
        tv_initialized = false;
        leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_DASH_REQUEST);
        return NULL;
    case EV_SCS_DE_FAILURE:
        vcu_data_p->scs_errors++;
        tv_initialized = false;
        leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_SCS_FAILURE);
        return NULL;
    case EV_SBS_SCS_DE_FAILURE:
        vcu_data_p->scs_errors++;
        tv_initialized = false;
        leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_SBS_SCS_FAILURE);
        return NULL;
    case EV_SBS_SCS_TV_FAILURE:
    case EV_SCS_TV_FAILURE:
        zcan_tx_scs(vcu_data_p->scs);
        vcu_data_p->scs_errors++;
        return NULL;
    case EV_1HZ_TICK:
        if (!tv_initialized) {
            ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_INIT);
            tv_initialized = true;
        }
        return NULL;
    case EV_100HZ_TICK:
        hsm_check_scs(scs_watchdog(&vcu_data_p->scs[0]));
        return NULL;
    }
}

// Drive Enable, Manual
// Handles:
// - Pedal implausabillities
// - TV (for Manual)
static void * super_de_manual(hsm_t * hsm_p, u8 ev) {
    static u8 prev_abpp = true;
    static u8 prev_app  = true;

    switch (ev) {
    default:
        return duper_de;
    case EV_ENTRY:
        hsm_dispatch(hsm_p, EV_PED_POS_UPD); // Force update of pedal positions
        zcan_tx_state();
        return duper_de;
    case EV_EXIT:
        return duper_de;
    case EV_ABPI_TIMEOUT: /* ABP Implausibility Triggered */
        if (vcu_data_p->status.abpp == false) {
            vcu_data_p->status.abpi = true;
            vcu_data_p->status.abpi_cnt++;
            zcan_tx_warning(vcu_Warning_1_0_ABP_IMPLAUSIBILITY, 0x00);
            zcan_tx_implausibilities();
        }
        return NULL;
    case EV_API_TIMEOUT: // AP  Implausibility Triggered
        if (vcu_data_p->status.app == false) {
            vcu_data_p->status.api = true;
            vcu_data_p->status.api_cnt++;
            zcan_tx_warning(vcu_Warning_1_0_AP_IMPLAUSIBILITY, 0x00);
            zcan_tx_implausibilities();
        }
        return NULL;
    case EV_PED_POS_UPD: {
        prev_abpp    = vcu_data_p->status.abpp;
        prev_app     = vcu_data_p->status.app;
        f32 throttle = get_throttle(vcu_data_p->meta_data.apps_l, vcu_data_p->meta_data.apps_r);
        /* Update implausibilities: */
        vcu_data_p->status.abpp = safety_acceleration_brake_pedal_plausibility(
            vcu_data_p->tv_input.hydraulic_brake_front,
            throttle,
            vcu_data_p->tv_input.power);
        vcu_data_p->status.app = safety_acceleration_pedal_plausibility(
            vcu_data_p->meta_data.apps_l,
            vcu_data_p->meta_data.apps_r);
        /* Handle Acceleration/Brake implausibility change: */
        if (vcu_data_p->status.abpp != prev_abpp) {
            if (vcu_data_p->status.abpp == false) {
                start_abpi_timer();
            } else {
                stop_abpi_timer();
            }
        }
        /* Handle Acceleration implausibility change: */
        if (vcu_data_p->status.app != prev_app) {
            if (vcu_data_p->status.app == false) {
                start_api_timer();
            } else {
                stop_api_timer();
            }
        }

        // Clear the implausibilities if the condition has cleared
        if (throttle <= 0.01) {
            vcu_data_p->status.abpi = false;
        }
        vcu_data_p->status.api = !(vcu_data_p->status.app);
        zcan_tx_implausibilities();

        vcu_data_p->tv_input.throttle = (throttle <= 0.06F || vcu_data_p->status.abpi || vcu_data_p->status.api) ? 0.0 : throttle;
        return NULL;
    }
    case EV_TX_TV_OUTPUT:
        if (scs_validate_tv(vcu_data_p) == SCS_TV_CRIT) {
            vcu_data_p->scs_errors++;
            zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SETPOINTS_SATURATED);
            zcan_tx_tv_output();
            hsm_transition(hsm_p, state_de_static);
        } else if (vcu_data_p->status.abpi || vcu_data_p->status.api) {
            inv_ctrl_zero(vcu_data_p);
            zcan_tx_inv_ctrl();
        } else {
            inv_ctrl_tv(vcu_data_p);
            zcan_tx_inv_ctrl();
        }
        return NULL;
    case EV_DRIVEMODE_REQ:
        // okay since we are already in de
        hsm_transition(hsm_p, state_de_selector);
        return NULL;
    }
}

// Drive Enable, Autonomous
// Handles:
// - Steering (EPOS)
// - TV is handled by duper_de
static void * super_de_as(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return duper_de;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_AS_DRIVING;
        zcan_tx_state();
        zcan_tx_ccc_start_max_cooling();
        return duper_de;
    case EV_EXIT:
        return duper_de;
#ifdef PU_TEST
#else
    case EV_START_CONTINUOUS_MONITORING:
        safety_continuous_monitoring(vcu_data_p, true);
        return NULL;
    case EV_CONTINUOUS_MONITORING_FAILED:
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
#endif
    case EV_PU_STEER_SETPOINT:
        epos_send_setpoint(vcu_data_p->pu_steering_setpoint);
        return NULL;
    case EV_UPDATE_ASSI:
        handle_assi(AS_driving);
        return NULL;
    case EV_ERROR_INVERTER_SETPOINTS_PU:
        zcan_tx_warning(vcu_Warning_1_0_AS_DRIVING_DENIED, vcu_Warning_1_0_SETPOINTS_SATURATED);
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
    case EV_AS_MISSION_FINISHED:
        // clang-format off
        if (vcu_data_p->tv_input.RPM_FL < AS_FINISHED_RPM_THRESHOLD &&
            vcu_data_p->tv_input.RPM_FR < AS_FINISHED_RPM_THRESHOLD &&
            vcu_data_p->tv_input.RPM_RL < AS_FINISHED_RPM_THRESHOLD &&
            vcu_data_p->tv_input.RPM_RR < AS_FINISHED_RPM_THRESHOLD) {

            hsm_transition(hsm_p, state_as_finished);
        }
        // clang-format on
        return NULL;
    case EV_AS_MISSION_FAILED:
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
    case EV_AS_FAILURE:
        zcan_tx_warning(vcu_Warning_1_0_AS_DRIVING_DENIED, vcu_Warning_1_0_SCS_FAILURE);
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
    case EV_TX_TV_OUTPUT:
        if (scs_validate_tv(vcu_data_p) == SCS_TV_CRIT) {
            vcu_data_p->scs_errors++;
            hsm_transition(hsm_p, state_as_emergency);
        } else {
            inv_ctrl_tv(vcu_data_p);
            zcan_tx_inv_ctrl();
        }
        return NULL;
    case EV_INV_DISABLED:
        hsm_transition(hsm_p, state_as_emergency);
        return NULL;
    }
}

// Drive Enable, Autonomous, LaunchControl
//
// for use with ControlMode::kLaunchSignal
static void * state_de_as_lc(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_as;
    case EV_ENTRY:
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_ENABLE_DV);
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_LC);
        zcan_tx_state();
        return super_de_as;
    case EV_EXIT:
        vcu_data_p->tv_input.as_Fx_requested = 0.f;
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_LC);
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_DISABLE_DV);
        return super_de_as;
    case EV_PU_LAUNCH_SIGNAL:
        // AS sends a bool, TV wants an Fx setpoint to start launch control
        // Launch control starts when as_Fx_requested goes over the
        // dv_high_trigger_lim, and stops when it goes under dv_low_trigger_lim
        vcu_data_p->tv_input.as_Fx_requested = (vcu_data_p->pu_launch_trigger) ? (vcu_data_p->tv_config.launch.dv_high_trigger_lim + 1.0) : (0.0);
        return NULL;
    }
}

// Drive Enable, Autonomous, Direct Control
//
// for use with ControlMode::kDirectInverterSetpoints
static void * state_de_as_direct(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_as;
    case EV_ENTRY:
        zcan_tx_state();
        return super_de_as;
    case EV_EXIT:
        return super_de_as;
    case EV_PU_INVERTER_SETPOINT:
        inv_ctrl_static(vcu_data_p);
        zcan_tx_inv_ctrl();
        return NULL;
    case EV_TX_TV_OUTPUT: // no TV
        scs_validate_tv(vcu_data_p); // Validate TV outputs to avoid going into emergency
        return NULL;
    }
}

// Drive Enable, Autonomous, Regular Control (TV)
//
// for use with ControlMode::kExternalTV
static void * state_de_as_rc(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_as;
    case EV_ENTRY:
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_ENABLE_DV);
        zcan_tx_state();
        return super_de_as;
    case EV_EXIT:
        vcu_data_p->tv_input.as_Fx_requested = 0.f;
        vcu_data_p->tv_input.as_Mz_requested = 0.f;
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_DISABLE_DV);
        return super_de_as;
    case EV_PU_FORCE_SETPOINT:
        vcu_data_p->tv_input.as_Fx_requested = vcu_data_p->pu_target_fx;
        vcu_data_p->tv_input.as_Mz_requested = vcu_data_p->pu_target_mz;
        return NULL;
    }
}

// immediately transitions to the appropriate state based on
// `vcu_data_p->config.mode`
static void * state_de_selector(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
    case EV_EXIT:
        return NULL;
    case EV_ENTRY:
#ifdef TV_HIL_TEST
        // Enable launch control in HIL-testing (wack)
        if (vcu_data_p->tv_config.mode_toggle.tv_control_method == 1) {
            hsm_transition(hsm_p, state_de_lc);
        } else {
            hsm_transition(hsm_p, state_de_rc);
        }
        return NULL;

#endif

        switch (vcu_data_p->config.mode) {
        case vcu_State_1_0_STATIC:
            hsm_transition(hsm_p, state_de_static);
            break;
        case vcu_State_1_0_REGULAR:
            hsm_transition(hsm_p, state_de_rc);
            break;
        case vcu_State_1_0_REGULAR_STATIC:
            hsm_transition(hsm_p, state_de_rc_static);
            break;
        case vcu_State_1_0_LAUNCH:
            hsm_transition(hsm_p, state_de_lc);
            break;
        case vcu_State_1_0_SKID:
            hsm_transition(hsm_p, state_de_skid);
            break;
        case vcu_State_1_0_AUTONOMOUS_DIRECT:
            hsm_transition(hsm_p, state_de_as_direct);
            break;
        case vcu_State_1_0_AUTONOMOUS_REGULAR:
            hsm_transition(hsm_p, state_de_as_rc);
            break;
        case vcu_State_1_0_AUTONOMOUS_LAUNCH:
            hsm_transition(hsm_p, state_de_as_lc);
            break;
        }
        return NULL;
    }
}

// Drive Enable, Static
// Sets torque equally on all wheels, bypassing the TV. Limited performance.
static void * state_de_static(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_manual;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_DE_STATIC;
        axi_timer_start(TIMER_INV, INVERTER_TIMEOUT_MSEC);
        zcan_tx_state();
        return NULL;
    case EV_EXIT:
        axi_timer_stop(TIMER_INV);
        return NULL;
    case EV_INV_TIMEOUT:
        if (vcu_data_p->status.abpi || vcu_data_p->status.api) {
            inv_ctrl_zero(vcu_data_p);
        } else {
            inv_ctrl_static(vcu_data_p);
        }
        zcan_tx_inv_ctrl();
        return NULL;
    case EV_TX_TV_OUTPUT:
        // dont use TV
        return NULL;
    }
}

// Drive Enable, Regular (use TV)
static void * state_de_rc(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_manual;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_DE_RC;
        zcan_tx_state();
        zcan_tx_ccc_start_max_cooling();
        return super_de_manual;
    case EV_EXIT:
        return super_de_manual;
    case EV_TX_TV_OUTPUT: // explicitly use TV
        return super_de_manual;
    case EV_SCS_TV_FAILURE:
        // duper_de handles incrementing vcu_data->scs_errors
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SCS_FAILURE);
        hsm_transition(hsm_p, state_de_static);
        return super_de_manual;
    case EV_SBS_SCS_TV_FAILURE:
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SBS_SCS_FAILURE);
        hsm_transition(hsm_p, state_de_static);
        return super_de_manual;
    }
}

// Drive Enable, Regular (use TV), Static (same torque on all wheels)
// In contrast to state_de_static, this /tells/ the TV to use static torque,
// rather than to manually set static torque setpoints.
// Gains: You can set high torque and rpm, because TV will do power limiting,
// instead of naïvely capping (max_rpm * max_torque), like in `state_de_static`
static void * state_de_rc_static(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_manual;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_DE_RC_STATIC;
        zcan_tx_state();
        zcan_tx_ccc_start_max_cooling();
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_STATIC);
        return NULL;
    case EV_EXIT:
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_STATIC);
        return NULL;
    case EV_TX_TV_OUTPUT: // explicitly use TV
        return super_de_manual;
    }
}

// Drive Enable, (Manual) Launch Control
static void * state_de_lc(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_manual;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_DE_LC;
        zcan_tx_state();
        zcan_tx_ccc_start_max_cooling();
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_LC);
        return NULL;
    case EV_EXIT:
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_LC);
        return NULL;
    case EV_TX_TV_OUTPUT: // explicitly use TV controls
        return super_de_manual;
    case EV_SCS_TV_FAILURE:
        // duper_de handles incrementing vcu_data->scs_errors
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SCS_FAILURE);
        hsm_transition(hsm_p, state_de_static);
        return super_de_manual;
    case EV_SBS_SCS_TV_FAILURE:
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SBS_SCS_FAILURE);
        hsm_transition(hsm_p, state_de_static);
        return super_de_manual;
    }
}

static void * state_de_skid(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return super_de_manual;
    case EV_ENTRY:
        vcu_data_p->status.cpu0.state = vcu_State_1_0_DE_SKID;
        zcan_tx_state();
        zcan_tx_ccc_start_max_cooling();
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_SKID);
        return NULL;
    case EV_EXIT:
        ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_TV_TOGGLE_SKID);
        return NULL;
    case EV_SCS_TV_FAILURE:
        // duper_de handles incrementing vcu_data->scs_errors
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SCS_FAILURE);
        hsm_transition(hsm_p, state_de_static);
        return super_de_manual;
    case EV_SBS_SCS_TV_FAILURE:
        zcan_tx_warning(vcu_Warning_1_0_LEAVING_TV, vcu_Warning_1_0_SBS_SCS_FAILURE);
        hsm_transition(hsm_p, state_de_static);
        return super_de_manual;
    }
}

// static limp, overrides state_de_static
static void * state_de_limp(hsm_t * hsm_p, u8 ev) {
    switch (ev) {
    default:
        return state_de_static;
    case EV_ENTRY:
    case EV_EXIT:
        return state_de_static;
    case EV_INV_TIMEOUT:
        if (vcu_data_p->status.abpi || vcu_data_p->status.api) {
            inv_ctrl_zero(vcu_data_p);
        } else {
            inv_ctrl_static_limp(vcu_data_p);
        }
        zcan_tx_inv_ctrl();
        return NULL;
    case EV_INV_READY:
        // clang-format off
        switch (vcu_data_p->inverter_type) {
        	default:
        		break;
            case INV_TYPE_I21:
                if (
                    (vcu_data_p->inv_state[FL] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[FL] == inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[FR] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[FR] == inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[RL] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[RL] == inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[RR] == inverter_State_1_0_STATE_ARMED || vcu_data_p->inv_state[RR] == inverter_State_1_0_STATE_ENABLED)
                    ) {
                    zcan_tx_warning(vcu_Warning_1_0_LEAVING_LIMP, vcu_Warning_1_0_INV_READY);
                    hsm_transition(hsm_p, state_de_selector);
                }
                break;
            case INV_TYPE_I19:
                if (
                    (vcu_data_p->inv_state[FL] == INV_STATE_READY) &&
                    (vcu_data_p->inv_state[FR] == INV_STATE_READY) &&
                    (vcu_data_p->inv_state[RL] == INV_STATE_READY) &&
                    (vcu_data_p->inv_state[RR] == INV_STATE_READY)
                    ) {
                    zcan_tx_warning(vcu_Warning_1_0_LEAVING_LIMP, vcu_Warning_1_0_INV_READY);
                    hsm_transition(hsm_p, state_de_selector);
                }
                break;
        }
        // clang-format on
        return NULL;
    case EV_INV_IDLE: // If the inverters are reset by the driver the vcu should detect this and enter drive enable
        inv_ctrl_enable_fw(vcu_data_p);
        return NULL;
    case EV_INV_DISABLED:
        zcan_tx_warning(vcu_Warning_1_0_INV_NOT_READY, vcu_Warning_1_0_INV_NOT_READY);
        // clang-format off
        switch (vcu_data_p->inverter_type) {
            default:
                leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_INV_NOT_READY);
                break;
            case INV_TYPE_I21:
                if (
                    (vcu_data_p->inv_state[FL] != inverter_State_1_0_STATE_ARMED && vcu_data_p->inv_state[FL] != inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[FR] != inverter_State_1_0_STATE_ARMED && vcu_data_p->inv_state[FR] != inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[RL] != inverter_State_1_0_STATE_ARMED && vcu_data_p->inv_state[RL] != inverter_State_1_0_STATE_ENABLED) &&
                    (vcu_data_p->inv_state[RR] != inverter_State_1_0_STATE_ARMED && vcu_data_p->inv_state[RR] != inverter_State_1_0_STATE_ENABLED)
                    ) {
                    leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_INV_NOT_READY);
                }
                break;
            case INV_TYPE_I19:
                if (
                    (vcu_data_p->inv_state[FL] != INV_STATE_READY) &&
                    (vcu_data_p->inv_state[FR] != INV_STATE_READY) &&
                    (vcu_data_p->inv_state[RL] != INV_STATE_READY) &&
                    (vcu_data_p->inv_state[RR] != INV_STATE_READY)
                    ) {
                    leave_de(hsm_p, vcu_Warning_1_0_LEAVING_DE, vcu_Warning_1_0_INV_NOT_READY);
                }
                break;
        }
        
        // clang-format on
        return NULL;
    }
}

// * disable inverter
// * idle second core (disable tv)
// * transition to idle/emergency
static void leave_de(hsm_t * hsm_p, u8 warning, u8 reason) {
    inv_ctrl_disable(vcu_data_p);
    zcan_tx_inv_ctrl();
    ipi_send(&ipi_i, IPI_APU1_ID, SWIRQ_IDLE);
    zcan_tx_warning(warning, reason);
    zcan_tx_scs(vcu_data_p->scs);
    hsm_transition_to_idle_or_emergency(hsm_p);
}

static void hsm_init(hsm_t * hsm_p, state_fp init_state) {
    hsm_p->state = init_state;
    hsm_dispatch(hsm_p, EV_ENTRY);
}

static void hsm_dispatch(hsm_t * hsm_p, u8 ev) {
    state_fp super = hsm_p->state;

    // relay the event to the super state until there is no super state
    do {
        super = (state_fp)(super)(hsm_p, ev);
    } while (super != NULL);
}

static void hsm_transition(hsm_t * hsm_p, state_fp new_state) {
    hsm_dispatch(hsm_p, EV_EXIT);
    hsm_p->state = new_state;
    hsm_dispatch(hsm_p, EV_ENTRY);
}

static void hsm_transition_to_idle_or_emergency(hsm_t * hsm_p) {
    // clang-format off
    if (vcu_State_1_0_MANUAL == vcu_data_p->config.mission) {
        hsm_transition(hsm_p, state_idle_manual);
    } else if (vcu_State_1_0_MISSION_SELECT != vcu_data_p->status.cpu0.state ||
               vcu_State_1_0_IDLE_AUTONOMOUS != vcu_data_p->status.cpu0.state ||
               vcu_State_1_0_IDLE_MANUAL != vcu_data_p->status.cpu0.state) {
        hsm_transition(hsm_p, state_as_emergency);
    } else {
        hsm_transition(hsm_p, state_idle_autonomous);
    }
    // clang-format on
}

static void clear_pu_setpoints() {
    vcu_data_p->pu_setpoints.rpm_req_fl = 0;
    vcu_data_p->pu_setpoints.rpm_req_fr = 0;
    vcu_data_p->pu_setpoints.rpm_req_rl = 0;
    vcu_data_p->pu_setpoints.rpm_req_rr = 0;

    vcu_data_p->pu_setpoints.trq_max_fl = 0;
    vcu_data_p->pu_setpoints.trq_max_fr = 0;
    vcu_data_p->pu_setpoints.trq_max_rl = 0;
    vcu_data_p->pu_setpoints.trq_max_rr = 0;
    
    vcu_data_p->pu_setpoints.trq_min_fl = 0;
    vcu_data_p->pu_setpoints.trq_min_fr = 0;
    vcu_data_p->pu_setpoints.trq_min_rl = 0;
    vcu_data_p->pu_setpoints.trq_min_rr = 0;

    vcu_data_p->pu_launch_trigger = 0;
    
    vcu_data_p->pu_target_fx = 0;
    vcu_data_p->pu_target_mz = 0;

    vcu_data_p->pu_steering_setpoint = 0;
}
