//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/hsm/hsm.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 24-04-19
#ifndef SRC_HSM_HSM_H_
#define SRC_HSM_HSM_H_

#include "xil_types.h"
#include "../utility/types.h"
#include "../safety/scs.h"
#include "../shared_src/ipi/ipi.h"

typedef enum {
    // Internal events:
    EV_ENTRY,
    EV_EXIT,
    // CAN hardware events:
    EV_RX_CANFD0,
    EV_RX_CANFD1,
    EV_RX_CANFD2,
    EV_RX_CAN_OPEN0,
    EV_RX_CAN_OPEN1,
    EV_RX_CAN_OPEN2,
    // INS events:
    EV_INS_RX_MSG,
    // Vehicle state events:
    EV_TS_OFF,
    EV_PED_POS_UPD,
    // UAVCAN service events:
    EV_REQ_ENTER_DE,
    EV_REQ_EXIT_DE,
    EV_REQ_KERS_ON,
    EV_REQ_KERS_OFF,
    EV_RTDS_GRANTED,
    EV_RTDS_DENIED,
    EV_RTDS_FINISHED,
    EV_DRIVEMODE_REQ,
    // AMP events:
    EV_SYNC_TV_INPUT,
    EV_SYNC_TV_OUTPUT,
    EV_TV_CONFIG_DONE,
    EV_SYNC_STATUS,
    EV_TV_CONFIG_START,
    // Inverter events:
    EV_INV_READY,
    EV_INV_DISABLED,
    EV_INV_IDLE,
    EV_REGISTER_I19,
    EV_REGISTER_I21,
    // SCS Events:
    EV_SCS_TV_FAILURE,
    EV_SCS_DE_FAILURE,
    EV_SBS_SCS_DE_FAILURE,
    EV_SBS_SCS_TV_FAILURE,
    EV_AS_FAILURE,
    // VCU system events:
    EV_TX_TV_OUTPUT,
    EV_TIMEOUT,
    EV_ABPI_TIMEOUT,
    EV_API_TIMEOUT,
    EV_INV_TIMEOUT,
    EV_DEBUG_DE,
    // Ticks
    EV_1HZ_TICK,
    EV_5HZ_TICK,
    EV_20HZ_TICK,
    EV_100HZ_TICK,
    EV_1000HZ_TICK,
    EV_10KILOHZ_TICK,
    // Mission select events:
    EV_REQ_NO_MISSION,
    EV_REQ_AUTONOMOUS_ACCELERATION,
    EV_REQ_AUTONOMOUS_SKID,
    EV_REQ_AUTONOMOUS_TRACKDRIVE,
    EV_REQ_EBS_TEST,
    EV_REQ_INSPECTION,
    EV_REQ_AUTONOMOUS_AUTOCROSS,
    EV_REQ_MANUAL,
    // TS events:
    EV_TSAB_EXTERNAL_BUTTON,
    EV_TSAB_COCKPIT_BUTTON,
    EV_ASMS_ON,
    EV_ASMS_OFF,
    // RES events:
    EV_RES_GO_PRESSED,
    // AS events
    EV_INITIAL_CHECKUP_SEQUENCE_SUCCESS,
    EV_INITIAL_CHECKUP_SEQUENCE_FAILED,
    EV_START_CONTINUOUS_MONITORING,
    EV_CONTINUOUS_MONITORING_FAILED,
    EV_UPDATE_ASSI,
    EV_AS_MISSION_FINISHED,
    EV_AS_MISSION_FAILED,
    EV_PU_FORCE_SETPOINT,    // force and moment: Fx and Mz
    EV_PU_INVERTER_SETPOINT, // raw setpoints
    EV_PU_STEER_SETPOINT,
    EV_PU_LAUNCH_SIGNAL,
    EV_ERROR_INVERTER_SETPOINTS_PU
} hsm_events_t;

void hsm_check_scs(scs_status_t status);

/**
 * @brief Runns the HSM, this is blocking
 * 
 * @param ipi_p pointer to the IPI instance
 * @param data_p pointer to the VCU data
 */
void hsm_run(vcu_data_t * p_data, XIpiPsu * ipi_p);
void hsm_add_ev(hsm_events_t id);
void hsm_add_ev_isr(hsm_events_t id);

#endif /* SRC_HSM_HSM_H_ */
