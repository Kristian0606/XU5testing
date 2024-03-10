/*
 * epos.c
 *
 *  Created on: 26 Feb 2022
 *      Author: DANKLORD420
 */

#include "remote_emergency_system.h"
#include "../utility/can_types.h"
#include "../utility/Revolve_CAN_definitions/Revolve_CAN_definitions.h"
#include "../zcan/zcan.h"

#define RES_NODE_ID           0x011 // Specifies the NMT NODE-ID, which is hopefully set to 1 by the DIP switch of the EPOS4.
#define RES_OPERATIONAL_STATE 1

static Can_message_t res_operational_message;

void remote_emergency_system_enable(void) {
    zcan_can_tx(res_operational_message);
}

static Can_message_t res_operational_message = {
    .idType     = STANDARD_CAN_ID,
    .messageID  = DV_NMT_NODE_CONTROL,
    .dataLength = 2,
    .data.u8[0] = RES_OPERATIONAL_STATE,
    .data.u8[1] = RES_NODE_ID
};
