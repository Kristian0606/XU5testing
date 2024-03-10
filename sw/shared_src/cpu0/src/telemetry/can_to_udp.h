/**
 * @file can_to_udp.h
 * @author Sivaranjith Sivarasa
 * @brief Translate a can frame to a udp package for sending over telemetry
 * @version 0.1
 * @date 2021-05-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef SRC_TELEMETRY_CAN_TO_UDP_H_
#define SRC_TELEMETRY_CAN_TO_UDP_H_

#include "xil_types.h"
#include "canard.h"

#define MAX_DATA_LEN  64U

#define HEADER_LENGTH (sizeof(int64_t) + sizeof(u32) + sizeof(u8))
#define MAX_MSG_LEN   (HEADER_LENGTH + MAX_DATA_LEN)

typedef union {
    u8 bytes[MAX_MSG_LEN];
    struct {
        u64 timestamp;
        u32 can_id;
        u8  data_length;
        u8  data[MAX_DATA_LEN];
    } fields;
} can_over_udp_msg_t;

/*
 * Translates can frame to udp msg
 *
 * @param[in] frame can frame to be translated
 * @param[out] udp_msg udp msg to be placed in buffer
 *
 * @return XST_SUCCESS
 */
void can_to_udp(const CanardFrame * frame, can_over_udp_msg_t * udp_msg, u32 * udp_length, u64 timestamp);
void udp_to_can(can_over_udp_msg_t * udp_msg, CanardFrame * can_frame);

#endif /* SRC_TELEMETRY_CAN_TO_UDP_H_ */
