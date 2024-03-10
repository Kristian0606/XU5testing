//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/ins/ins.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 17-04-19
#include "sleep.h"
#include "../shared_src/utility/asm.h"
#include "../safety/scs.h"
#include "../hsm/hsm.h"
#include "uart/uart.h"
#include "ins_cfg.h"
#include "stdlib.h"
#include "ins.h"

#define PI         3.14159265358979F
#define DEG2RAD(x) ((PI * x) / 180.0F)

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

    f32 roll_dt;
    f32 pitch_dt;
    f32 yaw_dt;
} pose_t;

static volatile u8 rx_buf[INS_BUF_SIZE] __attribute__((aligned(sizeof(void *) * 4)));
static volatile u8 msg_buf[INS_MSG_LEN] __attribute__((aligned(sizeof(void *) * 4)));
static volatile u8 get_msg_buf[INS_MSG_LEN] __attribute__((aligned(sizeof(void *) * 4)));
volatile u8        calib_in_progress = FALSE;

static void ins_isr(void * ref, u32 event, unsigned int event_data);
static f32  dt(f32 new_sample, f32 * samples);
static u16  calc_crc(volatile u8 data[], u8 len);

static pose_t ins_frame_to_vehicle_frame(pose_t ins_pose);

typedef enum {
    INS_INIT_CONFIG_UART,
    INS_INIT_NOT_STARTED_1,
    INS_INIT_NOT_STARTED_2,
    INS_INIT_NOT_STARTED_3,
    INS_INIT_NOT_STARTED_4,
    INS_INIT_NOT_STARTED_5,
    INS_INIT_NOT_STARTED_6,
    INS_INIT_INS_BAUDRATE_1,
    INS_INIT_INS_BAUDRATE_2,
    INS_INIT_INS_BAUDRATE_3,
    INS_INIT_INS_BAUDRATE_WAIT,
    INS_INIT_UART_BAUDRATE,
    INS_INIT_ANTENNA_BASELINE,
    INS_INIT_ANTENNA_OFFSET,
    INS_INIT_ASYNC_ON,
    INS_INIT_CONFIG_OUTPUT,
    INS_INIT_WAIT_RESPONSE,
    INS_INIT_ENABLE_IRQ,
} ins_init_stage_t;

static XScuGic *        intc_p;
static bool             ins_initiated = false;
static ins_init_stage_t init_stage    = INS_INIT_CONFIG_UART;

void ins_init(XScuGic * intc) {
    intc_p = intc;

    for (u8 i = 0; i < INS_BUF_SIZE; i++) {
        rx_buf[i] = 0x00;
    }
    // Initialize UART to INS_BAUD_DEFAULT.
    uart_init(intc_p, (XUartPs_Handler)ins_isr, INS_BAUD_DEFAULT);

    uart_set_baudrate(INS_BAUD);
    uart_enable_irq(intc_p);
    uart_rx(rx_buf, INS_MSG_LEN);
    ins_initiated = true;
}

void ins_continue_init() {
#warning "Temporarialy disabled, #eskildesign"
    /*switch(init_stage) {
        // 1) Wait until INS has booted
        case INS_INIT_NOT_STARTED_1: break;
        case INS_INIT_NOT_STARTED_2: break;
        case INS_INIT_NOT_STARTED_3: break;
        case INS_INIT_NOT_STARTED_4: break;
        case INS_INIT_NOT_STARTED_5: break;
        case INS_INIT_NOT_STARTED_6: break;

        // 2) Configure INS to desired baud rate.
        case INS_INIT_INS_BAUDRATE_1:
            uart_tx((u8 *)INS_CMD_SET_BAUD, INS_CMD_SET_BAUD_LEN);
        break;

        case INS_INIT_INS_BAUDRATE_2:
            uart_tx((u8 *)INS_CMD_SET_BAUD, INS_CMD_SET_BAUD_LEN);
        break;

        case INS_INIT_INS_BAUDRATE_3:
            uart_tx((u8 *)INS_CMD_SET_BAUD, INS_CMD_SET_BAUD_LEN); // Ugly, but needed to set baudrate.
        break;

        case INS_INIT_INS_BAUDRATE_WAIT: break;

        // 3) Update uart baudrate & set asynchronous message type to none. - This does NOT affect binary output messages.
        case INS_INIT_UART_BAUDRATE:
            uart_set_baudrate(INS_BAUD);
            uart_tx((u8 *)INS_CMD_ASYNC_NONE, INS_CMD_ASYNC_NONE_LEN);
        break;

        // 4) Set Antenna Locations:
        case INS_INIT_ANTENNA_BASELINE:
            uart_tx((u8 *)INS_CMD_COMPASS_BASELINE, INS_CMD_COMPASS_BASELINE_LEN);
        break;

        case INS_INIT_ANTENNA_OFFSET:
            uart_tx((u8 *)INS_CMD_ANT_A_OFFSET, INS_CMD_ANT_A_OFFSET_LEN);
        break;

        // 5) Enable asynchronous messages. *    - Required for binary output messages.
        case INS_INIT_ASYNC_ON:
            uart_tx((u8 *)INS_CMD_ASYNC_ON, INS_CMD_ASYNC_ON_LEN);
        break;

        // 6) Configure the binary output messages. - See ins_cfg.h for it's description.
        case INS_INIT_CONFIG_OUTPUT:
            uart_tx((u8 *)INS_CMD_SET_MSG, INS_CMD_SET_MSG_LEN);
        break;

        // 7) Wait for all responses to arrive before fully initializing UART. - This clears the RX and TX FIFOs.
        case INS_INIT_WAIT_RESPONSE: break;

        // 8) Enable the UART interrupts.
        case INS_INIT_ENABLE_IRQ:
            uart_enable_irq(intc_p);
            uart_rx(rx_buf, INS_MSG_LEN);
            ins_initiated = true;
        break;

        default: break;
    }

    init_stage++;*/
}

bool ins_is_inited() {
    return ins_initiated;
}

void ins_get_msg(vcu_data_t * vcu_data_p) {
    if (!ins_initiated) {
        return;
    }

    vcu_data_p->status.ins_rx_cnt++;
    scs_status_t status = SCS_OK;

    ENTER_CRITICAL();
    for (u8 i = 0; i < INS_MSG_LEN; i++) {
        get_msg_buf[i] = msg_buf[i];
    }
    LEAVE_CRITICAL();

    if (calc_crc(get_msg_buf, INS_MSG_LEN) != 0x0000) {
        uart_rx(rx_buf, INS_MSG_LEN);
        vcu_data_p->status.ins_crc_er++;

        return;
    }

    // Transform to float values from u8 array
    pose_t ins = {
        .vx       = *(f32 *)(&get_msg_buf[INS_VX]),
        .vy       = *(f32 *)(&get_msg_buf[INS_VY]),
        .vz       = *(f32 *)(&get_msg_buf[INS_VZ]),
        .ax       = *(f32 *)(&get_msg_buf[INS_AX]),
        .ay       = *(f32 *)(&get_msg_buf[INS_AY]),
        .az       = *(f32 *)(&get_msg_buf[INS_AZ]),
        .roll     = *(f32 *)(&get_msg_buf[INS_ROLL]),
        .pitch    = *(f32 *)(&get_msg_buf[INS_PITCH]),
        .yaw      = *(f32 *)(&get_msg_buf[INS_YAW]),
        .roll_dt  = *(f32 *)(&get_msg_buf[INS_ROLL_RATE]),
        .pitch_dt = *(f32 *)(&get_msg_buf[INS_PITCH_RATE]),
        .yaw_dt   = *(f32 *)(&get_msg_buf[INS_YAW_RATE]),
    };

    pose_t vehicle = ins_frame_to_vehicle_frame(ins);
    vehicle.roll -= vcu_data_p->config.roll_zero;
    vehicle.pitch -= vcu_data_p->config.pitch_zero;

    pose_t ins_raw = {
        .vx       = 0,
        .vy       = 0,
        .vz       = 0,
        .ax       = *(f32 *)(&get_msg_buf[INS_AX_RAW]),
        .ay       = *(f32 *)(&get_msg_buf[INS_AY_RAW]),
        .az       = *(f32 *)(&get_msg_buf[INS_AZ_RAW]),
        .roll     = *(f32 *)(&get_msg_buf[INS_ROLL_STD]),
        .pitch    = *(f32 *)(&get_msg_buf[INS_PITCH_STD]),
        .yaw      = *(f32 *)(&get_msg_buf[INS_YAW_STD]),
        .roll_dt  = *(f32 *)(&get_msg_buf[INS_ROLL_RATE_RAW]),
        .pitch_dt = *(f32 *)(&get_msg_buf[INS_PITCH_RATE_RAW]),
        .yaw_dt   = *(f32 *)(&get_msg_buf[INS_YAW_RATE_RAW]),
    };

    pose_t vehicle_raw = ins_frame_to_vehicle_frame(ins_raw);
    vehicle_raw.roll   = fabsf(vehicle_raw.roll);  // Absolute value for std,
    vehicle_raw.pitch  = fabsf(vehicle_raw.pitch); // to fix potential change of sign
    vehicle_raw.yaw    = fabsf(vehicle_raw.yaw);   // when performing rotation to vehicle frame

    vcu_data_p->meta_data.ax_raw           = vehicle_raw.ax;
    vcu_data_p->meta_data.ay_raw           = vehicle_raw.ay;
    vcu_data_p->meta_data.az_raw           = vehicle_raw.az;
    vcu_data_p->meta_data.roll_rate_raw    = vehicle_raw.roll_dt;
    vcu_data_p->meta_data.pitch_rate_raw   = vehicle_raw.pitch_dt;
    vcu_data_p->meta_data.yaw_rate_raw     = vehicle_raw.yaw_dt;
    vcu_data_p->meta_data.time_gps_lsb     = *(u32 *)(&get_msg_buf[INS_TIME_GPS]);
    vcu_data_p->meta_data.time_gps_msb     = *(u32 *)(&get_msg_buf[INS_TIME_GPS + 4]);
    vcu_data_p->meta_data.time_gps_pps_lsb = *(u32 *)(&get_msg_buf[INS_TIME_GPS_PPS]);
    vcu_data_p->meta_data.time_gps_pps_msb = *(u32 *)(&get_msg_buf[INS_TIME_GPS_PPS + 4]);
    vcu_data_p->meta_data.time_status      = *(u8 *)(&get_msg_buf[INS_TIME_STATUS]);
    vcu_data_p->meta_data.yaw_std          = vehicle_raw.yaw;
    vcu_data_p->meta_data.pitch_std        = vehicle_raw.pitch;
    vcu_data_p->meta_data.roll_std         = vehicle_raw.roll;
    vcu_data_p->meta_data.pos_std          = *(f32 *)(&get_msg_buf[INS_POS_STD]);
    vcu_data_p->meta_data.vel_std          = *(f32 *)(&get_msg_buf[INS_VEL_STD]);

    vcu_data_p->ins_data.vx         = scs_validate(vehicle.vx, &(vcu_data_p->scs[SCS_VX]), &status);
    vcu_data_p->ins_data.vy         = scs_validate(vehicle.vy, &(vcu_data_p->scs[SCS_VY]), &status);
    vcu_data_p->ins_data.vz         = scs_validate(vehicle.vz, &(vcu_data_p->scs[SCS_VZ]), &status);
    vcu_data_p->ins_data.ax         = scs_validate(vehicle.ax, &(vcu_data_p->scs[SCS_AX]), &status);
    vcu_data_p->ins_data.ay         = scs_validate(vehicle.ay, &(vcu_data_p->scs[SCS_AY]), &status);
    vcu_data_p->ins_data.az         = scs_validate(vehicle.az, &(vcu_data_p->scs[SCS_AZ]), &status);
    vcu_data_p->ins_data.yaw_rate   = scs_validate(vehicle.yaw_dt, &(vcu_data_p->scs[SCS_YAW_DT]), &status);
    vcu_data_p->ins_data.roll       = scs_validate(vehicle.roll, &(vcu_data_p->scs[SCS_ROLL]), &status);
    vcu_data_p->ins_data.roll_rate  = scs_validate(vehicle.roll_dt, &(vcu_data_p->scs[SCS_ROLL_DT]), &status);
    vcu_data_p->ins_data.pitch      = scs_validate(vehicle.pitch, &(vcu_data_p->scs[SCS_PITCH]), &status);
    vcu_data_p->ins_data.pitch_rate = scs_validate(vehicle.pitch_dt, &(vcu_data_p->scs[SCS_PITCH_DT]), &status);

    vcu_data_p->ins_data.yaw = vehicle.yaw;

    static f32 roll_rate_samples[3]  = { 0 };
    static f32 pitch_rate_samples[3] = { 0 };
    static f32 yaw_rate_samples[3]   = { 0 };

    vcu_data_p->ins_data.roll_rate_dt  = dt(vcu_data_p->ins_data.roll_rate, roll_rate_samples);
    vcu_data_p->ins_data.pitch_rate_dt = dt(vcu_data_p->ins_data.pitch_rate, pitch_rate_samples);
    vcu_data_p->ins_data.yaw_rate_dt   = scs_validate(dt(vcu_data_p->ins_data.yaw_rate, yaw_rate_samples), &(vcu_data_p->scs[SCS_YAW_DTDT]), &status);

    ins_to_tv_input(vcu_data_p);

    vcu_data_p->meta_data.yaw         = vehicle.yaw;
    vcu_data_p->meta_data.gnss_a_sats = *(u8 *)(&get_msg_buf[INS_GNSS_A_SATS]);
    vcu_data_p->meta_data.gnss_a_fix  = *(u8 *)(&get_msg_buf[INS_GNSS_A_FIX]);
    vcu_data_p->meta_data.gnss_b_sats = *(u8 *)(&get_msg_buf[INS_GNSS_B_SATS]);
    vcu_data_p->meta_data.gnss_b_fix  = *(u8 *)(&get_msg_buf[INS_GNSS_B_FIX]);
    vcu_data_p->meta_data.status_ins  = *(u16 *)(&get_msg_buf[INS_INS_STATUS]);
    vcu_data_p->meta_data.ahrs_status = *(u16 *)(&get_msg_buf[INS_AHRS_STATUS]);

    ins_f64_conversion_t latitude;
    ins_f64_conversion_t longitude;
    ins_f64_conversion_t altitude;
    for (u8 i = 0; i < 8; i++) {
        latitude.uint8_array[i]  = get_msg_buf[INS_LAT + i];
        longitude.uint8_array[i] = get_msg_buf[INS_LON + i];
        altitude.uint8_array[i]  = get_msg_buf[INS_ALT + i];
    }
    // Read Back the Union as a float64
    vcu_data_p->meta_data.lat = latitude.float64;
    vcu_data_p->meta_data.lon = longitude.float64;
    vcu_data_p->meta_data.alt = altitude.float64;

    u16 filter_status                = vcu_data_p->meta_data.status_ins & INS_FIL_MASK;
    vcu_data_p->tv_input.ins_aligned = filter_status == INS_FIL_TRACKING;

    hsm_check_scs(status);
}

void ins_to_tv_input(vcu_data_t * vcu_data_p) {

    vcu_data_p->tv_input.vx          = vcu_data_p->ins_data.vx;
    vcu_data_p->tv_input.vy          = vcu_data_p->ins_data.vy;
    vcu_data_p->tv_input.vz          = vcu_data_p->ins_data.vz;
    vcu_data_p->tv_input.ax          = vcu_data_p->ins_data.ax;
    vcu_data_p->tv_input.ay          = vcu_data_p->ins_data.ay;
    vcu_data_p->tv_input.az          = vcu_data_p->ins_data.az;
    vcu_data_p->tv_input.yaw_rate    = vcu_data_p->ins_data.yaw_rate;
    vcu_data_p->tv_input.roll        = DEG2RAD(vcu_data_p->ins_data.roll);
    vcu_data_p->tv_input.roll_rate   = vcu_data_p->ins_data.roll_rate;
    vcu_data_p->tv_input.pitch       = DEG2RAD(vcu_data_p->ins_data.pitch);
    vcu_data_p->tv_input.pitch_rate  = vcu_data_p->ins_data.pitch_rate;
    vcu_data_p->tv_input.yaw_rate_dt = vcu_data_p->ins_data.yaw_rate_dt;
}

void ins_tx_send_rtcm(u8 * buffer, u16 buffer_size) {
    if (!ins_initiated) {
        return;
    }

    uart_tx(buffer, buffer_size);
}

static void ins_isr(void * ref, u32 event, unsigned int event_data) {
    hsm_add_ev_isr(EV_INS_RX_MSG);
    memcpy((u32 *)msg_buf, (u32 *)rx_buf, INS_MSG_LEN);
    uart_rx(rx_buf, INS_MSG_LEN);
}

/* CRC16-CCITT */
static u16 calc_crc(volatile u8 data[], u8 len) {
    u16 crc = 0;

    for (u8 i = 1; i < len; i++) {
        crc = (u8)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (u8)(crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00FF) << 5;
    }

    return crc;
}

/*
 * Three-point one-sided differentiator:
 *     y'(x) = -3*y(x) + 4*y(x+h) - y(x+2h) / 2*h
 * where
 *     h = -2.5 ms
 *
 * Input EMA filter:
 *     filtered = new * a + (1 - a) * prev_filtered
 * where
 *     a = 0.8
 */
static f32 dt(f32 new_sample, f32 * samples) {
    f32 * y = samples;

    y[0] = y[1];
    y[1] = y[2];
    y[2] = new_sample * 0.6 + (1 - 0.6) * y[2];

    return (-3.0 * y[2] + 4.0 * y[1] - y[0]) / (2.0 * -0.0025);
}

// Magic rotation matrix incomming :D
// Transforms the INS NED frame to the vehicle ISO frame
// TODO: Remember to update this in the future if the INS is rotated or something like that
static pose_t ins_frame_to_vehicle_frame(pose_t ins) {
    pose_t vehicle;

    vehicle.vx = ins.vx;
    vehicle.vy = -ins.vy;
    vehicle.vz = -ins.vz;

    vehicle.ax = ins.ax;
    vehicle.ay = -ins.ay;
    vehicle.az = -ins.az;

    vehicle.roll  = ins.roll;
    vehicle.pitch = -ins.pitch;
    vehicle.yaw   = -ins.yaw;

    vehicle.roll_dt  = ins.roll_dt;
    vehicle.pitch_dt = -ins.pitch_dt;
    vehicle.yaw_dt   = -ins.yaw_dt;

    return vehicle;
}
