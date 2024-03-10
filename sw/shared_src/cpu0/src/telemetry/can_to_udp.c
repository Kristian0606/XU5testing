/**
 * @file can_to_udp.c
 * @author Sivaranjith Sivarasa
 * @brief Translate a can frame to a udp package for sending over telemetry
 * @version 0.1
 * @date 2021-05-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "can_to_udp.h"
#include <string.h>
#include "xstatus.h"
#include "../utility/conversions.h"

void can_to_udp(const CanardFrame * frame, can_over_udp_msg_t * udp_msg, u32 * udp_length, u64 timestamp) {
    udp_msg->fields.timestamp   = swap_endian_64(timestamp);
    udp_msg->fields.can_id      = swap_endian_32(frame->extended_can_id);
    udp_msg->fields.data_length = (u8)frame->payload_size;

    memcpy(udp_msg->fields.data, frame->payload, frame->payload_size);

    *udp_length = HEADER_LENGTH + frame->payload_size;
}

void udp_to_can(can_over_udp_msg_t * udp_msg, CanardFrame * can_frame) {
    udp_msg->fields.timestamp = swap_endian_64(udp_msg->fields.timestamp);
    udp_msg->fields.can_id    = swap_endian_32(udp_msg->fields.can_id);

    can_frame->extended_can_id = udp_msg->fields.can_id;
    can_frame->payload_size    = udp_msg->fields.data_length;
    can_frame->payload         = udp_msg->fields.data;
}
