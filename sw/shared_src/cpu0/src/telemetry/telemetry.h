/**
 * @file telemetry.h
 * @author Sivaranjith Sivarasa
 * @brief Module for sending and receiving can frames over udp, also known as telemetry
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef SRC_TELEMETRY_TELEMETRY_H_
#define SRC_TELEMETRY_TELEMETRY_H_

#include "lwip/inet.h"
#include "xil_types.h"
#include "can_to_udp.h"

#define DEFAULT_IP_ADDRESS            "10.19.65.102"
#define DEFAULT_IP_MASK               "255.255.0.0"
#define DEFAULT_GW_ADDRESS            "10.19.65.1"

#define BROADCAST_IP_ADDRESS          "10.19.255.255"

#define UDP_CONN_PORT_ANALYZE_SEND    1234 // Old broadcast.
#define UDP_CONN_PORT_ANALYZE_RECEIVE 1235
#define UDP_CONN_PORT_PU_RECEIVE      1221
#define UDP_CONN_PORT_PU_SEND         1220

#define UDP_SEND_BUFSIZE              1440

#define ANALYZE_BUFFER_LEN            (72 + 1 + 4)
#define PU_BUFFER_LEN                 64
#define TM_FILL_PERCENTAGE            (0.9) // How full the buffer has to be before it is sent

/**
 * @brief Initializes the telemetry module
 * 
 */
void telemetry_init();

/**
 * @brief Continues the initialization of the telemetry module
 * 
 */
void telemetry_continue_init();

/**
 * @brief Checks if the ethernet is up
 * 
 * @return true if the ethernet is up
 * @return false if the ethernet is down
 */
bool telemetry_is_up();
void telemetry_poll_ethernet();
u8   telemetry_add_to_buffer(can_over_udp_msg_t * udp_msg, u32 udp_len);
u8   telemetry_send_to_pu(can_over_udp_msg_t * udp_msg, u32 udp_len);
u8   telemetry_get_buffer_filled();

#endif /* SRC_TELEMETRY_TELEMETRY_H_ */
