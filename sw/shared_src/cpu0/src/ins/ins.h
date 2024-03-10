//   _____ _____ _____
//  |   | |__   |   | | cpu0/ins/ins.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 17-04-19
#ifndef SRC_INS_INS_H_
#define SRC_INS_INS_H_
#include "../shared_src/intc/intc.h"
#include "../utility/types.h"

/**
 * @brief Initializes the INS.
 * 
 * @param intc_p pointer to the interrupt controller instance
 */
void ins_init(XScuGic * intc_p);

/**
 * @brief Doesent do anything.
 * 
 */
void ins_continue_init();

/**
 * @brief Checks if the INS is initialized.
 * @return true if the INS is initialized, false otherwise
 */
bool ins_is_inited();

/**
 * @brief Gets the INS data transformes into float for the VCU data struct.
 * 
 * @param vcu_data_p pointer to the VCU data struct
 */
void ins_get_msg(vcu_data_t * vcu_data_p);

/**
 * @brief Sends the RTCM message. Not Used.
 * 
 * @param buffer pointer to the buffer
 * @param buffer_size the size of the buffer
 */
void ins_tx_send_rtcm(u8 * buffer, u16 buffer_size);

/**
 * @brief Moves the INS data to the TV input.
 * 
 * @param vcu_data_p pointer to the VCU data struct
 */
void ins_to_tv_input(vcu_data_t * vcu_data_p);

typedef union {
    u8  uint8_array[8];
    f64 float64;
} ins_f64_conversion_t;

#endif /* SRC_INS_INS_H_ */
