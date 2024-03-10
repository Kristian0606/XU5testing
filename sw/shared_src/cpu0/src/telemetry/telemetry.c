/**
 * @file telemetry.c
 * @author Sivaranjith Sivarasa
 * @brief Module for sending and receiving can frames over udp, also known as telemetry
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "../ins/ins.h"
#include "../utility/log.h"
#include "../time/time.h"
#include "../timer/ttc_timer.h"
#include "../zcan/zcan.h"
#include "../hsm/hsm.h"
#include "can_to_udp.h"
#include "telemetry.h"
#include "canard.h"

#include <stdio.h>
#include "lwip/priv/tcp_priv.h"
#include "netif/xadapter.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "xil_cache.h"
#include "lwip/udp.h"
#include "lwipopts.h"
#include "xstatus.h"
#include "../utility/conversions.h"

static struct udp_pcb * pcb_pu;
static struct udp_pcb * pcb_analyze;
struct netif            server_netif;
struct netif *          netif;
static char             send_buf[UDP_SEND_BUFSIZE];
static uint32_t         buffer_tail = 0;
static ip_addr_t        remote_addr;

static void assign_default_ip(ip_addr_t * ip, ip_addr_t * mask, ip_addr_t * gw);
static void telemetry_init_all_pcbs(void);
static void telemetry_udp_receive_pu(void * arg, struct udp_pcb * tpcb, struct pbuf * p, const ip_addr_t * addr, u16_t port);
static void telemetry_udp_receive_analyze(void * arg, struct udp_pcb * tpcb, struct pbuf * p, const ip_addr_t * addr, u16_t port);
static u8   telemetry_send_buffer();

/* the mac address of the board. this should be unique per board */
const unsigned char MAC_ADDRESS[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

static bool ethernet_up = false;

static void assign_default_ip(ip_addr_t * ip, ip_addr_t * mask, ip_addr_t * gw) {
    int err;

    log_telemetry("Configuring default IP %s\r\n", DEFAULT_IP_ADDRESS);

    err = inet_aton(DEFAULT_IP_ADDRESS, ip);
    if (!err) {
        log_telemetry_error("Invalid default IP address: %d\r\n", err);
    }

    err = inet_aton(DEFAULT_IP_MASK, mask);
    if (!err) {
        log_telemetry_error("Invalid default IP MASK: %d\r\n", err);
    }

    err = inet_aton(DEFAULT_GW_ADDRESS, gw);
    if (!err) {
        log_telemetry_error("Invalid default gateway address: %d\r\n", err);
    }

    err = inet_aton(BROADCAST_IP_ADDRESS, &remote_addr);
    if (!err) {
        log_telemetry_error("Invalid broadcast IP address: %d\r\n", err);
    }
}

void telemetry_init() {
    netif = &server_netif;

    lwip_init();

    telemetry_continue_init();
}

void telemetry_continue_init() {
    /* Add network interface to the netif_list, and set it as default */
    if (!xemac_add(netif, NULL, NULL, NULL, MAC_ADDRESS, XPAR_XEMACPS_0_BASEADDR)) {
        log_telemetry_error("Error adding N/W interface\r\n");
        return;
    }

    eth_link_detect(netif);

    if (!(netif->flags & NETIF_FLAG_LINK_UP)) {
        ethernet_up = false;
        return;
    }
    netif_set_default(netif);

    netif_set_up(netif);

    assign_default_ip(&(netif->ip_addr), &(netif->netmask), &(netif->gw));

    log_init("\r\n Started Telemetry \r\n");

    /* start the application*/
    telemetry_init_all_pcbs();

    ethernet_up = true;
}

bool telemetry_is_up() {
    return ethernet_up;
}

/// poll underlying ethernet for new packets
///
/// this must be regularly polled to pass us new packets from the internal
/// buffer, frequency dependant on ethernet traffic and required latency
///
/// doc: https://github.com/Xilinx/embeddedsw/blob/master/ThirdParty/sw_services/lwip211/doc/lwip211.pdf
/// - see "RAW mode"
/// lwip examples: https://www.xilinx.com/content/dam/xilinx/support/documents/application_notes/xapp1026.pdf
/// - see "Creating an lwIP application using the RAW API"
void telemetry_poll_ethernet() {
    if (!ethernet_up) {
        return;
    }

    // xemacif_input() will check the emac for new packets, and if any,
    // transfer them to the lwIP-stack
    // see XAPP1026
    xemacif_input(netif);
}

u8 telemetry_add_to_buffer(can_over_udp_msg_t * udp_msg, u32 udp_len) {
    if (!ethernet_up) {
        return XST_FAILURE;
    }

    if (buffer_tail + udp_len >= (u32)(TM_FILL_PERCENTAGE * UDP_SEND_BUFSIZE)) {
        telemetry_send_buffer();
    }

    memcpy(&send_buf[buffer_tail], udp_msg->bytes, udp_len);
    buffer_tail += udp_len;
    return XST_SUCCESS;
}

/// @brief send telemetry data over udp
/// @param dst_pcb where to send
/// @param dst_port what port to send on
/// @param data_src pointer to payload
/// @param data_length length of payload
/// @return XST_SUCCESS on success, XST_FAILURE on failure
static u8 telemetry_send_helper(struct udp_pcb * dst_pcb, u16_t dst_port, const void * data_src, u16_t data_length) {
    if (!ethernet_up) {
        return XST_FAILURE;
    }

    struct pbuf * packet = pbuf_alloc(PBUF_TRANSPORT, data_length, PBUF_POOL);

    if (!packet) {
        log_telemetry_error("error allocating pbuf\r\n");
        return XST_FAILURE;
    } else {
        memcpy(packet->payload, data_src, data_length);
    }

    err_t err = udp_sendto(dst_pcb, packet, &remote_addr, dst_port);
    pbuf_free(packet);

    if (err != ERR_OK) {
        log_telemetry_error("Failed to send UDP packet\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/// @brief Sends the current buffer to analyze
/// Is automatically invoked by telemetry_add_to_buffer() when the buffer is starting to get full
static u8 telemetry_send_buffer() {
    // no ethernet_up check, the helper does it
    err_t err   = telemetry_send_helper(pcb_analyze, UDP_CONN_PORT_ANALYZE_SEND, send_buf, buffer_tail);
    buffer_tail = 0; // should we perhaps only clear if we managed to send? (this would change behavior)
    return err;
}

/// @brief Sends the passed message to the pu over udp
u8 telemetry_send_to_pu(can_over_udp_msg_t * udp_msg, u32 udp_len) {
    // no ethernet_up check, the helper does it
    err_t err = telemetry_send_helper(pcb_pu, UDP_CONN_PORT_PU_SEND, udp_msg, udp_len);
    return err;
}

/// @returns How full the buffer is on a scale 0-100 inclusive
u8 telemetry_get_buffer_filled() {
    return (u8)((100 * buffer_tail) / UDP_SEND_BUFSIZE);
}

/// @brief Inits a new PCB by binding a port and setting a callback
/// Note that in this context **PCB is not printed circuit board, but Protocol Control Block**.
static void telemetry_init_pcb(struct udp_pcb * pcb, u16_t port, udp_recv_fn callback) {
    err_t err = udp_bind(pcb, IPADDR_ANY, port);
    if (err != ERR_OK) {
        log_telemetry_error("Failed to init pcb at port %u\r\n", port);
        udp_remove(pcb);
    } else {
        udp_recv(pcb, callback, NULL);
    }
}

/// @brief Inits the PCBs and sets callbacks
static void telemetry_init_all_pcbs(void) {
    pcb_pu = udp_new();
    if (pcb_pu) {
        telemetry_init_pcb(pcb_pu, UDP_CONN_PORT_PU_RECEIVE, telemetry_udp_receive_pu);
    }

    pcb_analyze = udp_new();
    if (pcb_analyze) {
        telemetry_init_pcb(pcb_analyze, UDP_CONN_PORT_ANALYZE_RECEIVE, telemetry_udp_receive_analyze);
    }
}

static void telemetry_udp_receive_pu(void * arg, struct udp_pcb * tpcb, struct pbuf * p, const ip_addr_t * addr, u16_t port) {
    CanardFrame        frame;
    can_over_udp_msg_t udp_msg;
    u64                timestamp;
    get_time_us(&timestamp);

    // TODO: consider checking if they match exactly. I think it has to be
    // exactly the same size, but I am not sure. At least we don't overflow the
    // buffer.

    if (sizeof(udp_msg.bytes) >= p->len) {
        memcpy(udp_msg.bytes, p->payload, p->len);
        udp_msg.fields.timestamp = swap_endian_64(timestamp);
        telemetry_add_to_buffer(&udp_msg, p->len);

        udp_to_can(&udp_msg, &frame);
        zcan_rx_frame(&frame);
    } else {
        log_telemetry_error("Received PU packet too large for buffer\r\n");
        // TODO: increment error counter
    }

    pbuf_free(p);
}

static void telemetry_udp_receive_analyze(void * arg, struct udp_pcb * tpcb, struct pbuf * p, const ip_addr_t * addr, u16_t port) {
    CanardFrame        frame;
    can_over_udp_msg_t udp_msg;

    if (sizeof(udp_msg.bytes) >= p->len) {
        memcpy(udp_msg.bytes, p->payload, p->len);
        udp_to_can(&udp_msg, &frame);
        zcan_rx_frame(&frame);
    } else {
        log_telemetry_error("Received analyze packet too large for buffer\r\n");
        // TODO: increment error counter
    }

    pbuf_free(p);
}