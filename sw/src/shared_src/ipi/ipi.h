/*
 * ipi.h
 *
 *  Created on: Nov 18, 2020
 *      Author: Eskil
 */

#ifndef SRC_DRIVERS_IPI_IPI_H_
#define SRC_DRIVERS_IPI_IPI_H_

#include "xipipsu.h"
#include "xparameters.h"
#include "xil_types.h"

#define IPI_APU0                    XPAR_PSU_IPI_0_BIT_MASK 
#define IPI_RPU0                    XPAR_PSU_IPI_1_BIT_MASK
#define IPI_RPU1                    XPAR_PSU_IPI_2_BIT_MASK
#define IPI_PMU0                    XPAR_PSU_IPI_3_BIT_MASK
#define IPI_PMU1                    XPAR_PSU_IPI_4_BIT_MASK
#define IPI_PMU2                    XPAR_PSU_IPI_5_BIT_MASK
#define IPI_PMU3                    XPAR_PSU_IPI_6_BIT_MASK
#define IPI_APU1                    XPAR_PSU_IPI_7_BIT_MASK
// #define IPI_PL1                     XPAR_PSU_IPI_8_BIT_MASK
// #define IPI_PL2                     XPAR_PSU_IPI_9_BIT_MASK
// #define IPI_PL3                     XPAR_PSU_IPI_10_BIT_MASK

#define IPI_APU0_ID                  XPAR_PSU_IPI_0_DEVICE_ID
#define IPI_APU1_ID                  XPAR_PSU_IPI_7_DEVICE_ID
// #define IPI_RPU0_ID                 XPAR_PSU_IPI_1_DEVICE_ID
// #define IPI_RPU1_ID                 XPAR_PSU_IPI_2_DEVICE_ID
// #define IPI_PMU0_ID                 XPAR_PSU_IPI_3_DEVICE_ID
// #define IPI_PMU1_ID                 XPAR_PSU_IPI_4_DEVICE_ID
// #define IPI_PMU2_ID                 XPAR_PSU_IPI_5_DEVICE_ID
// #define IPI_PMU3_ID                 XPAR_PSU_IPI_6_DEVICE_ID
// #define IPI_PL0_ID                  XPAR_PSU_IPI_7_DEVICE_ID
// #define IPI_PL1_ID                  XPAR_PSU_IPI_8_DEVICE_ID
// #define IPI_PL2_ID                  XPAR_PSU_IPI_9_DEVICE_ID
// #define IPI_PL3_ID                  XPAR_PSU_IPI_10_DEVICE_ID


#define IPI_APU0_INT_ID                  XPAR_PSU_IPI_0_INT_ID
#define IPI_APU1_INT_ID                  XPAR_PSU_IPI_7_INT_ID

// #define IPI_1_INT_ID                  XPAR_PSU_IPI_1_INT_ID
// #define IPI_2_INT_ID                  XPAR_PSU_IPI_2_INT_ID
// #define IPI_3_INT_ID                  XPAR_PSU_IPI_3_INT_ID
// #define IPI_4_INT_ID                  XPAR_PSU_IPI_4_INT_ID
// #define IPI_5_INT_ID                  XPAR_PSU_IPI_5_INT_ID
// #define IPI_6_INT_ID                  XPAR_PSU_IPI_6_INT_ID
// #define IPI_7_INT_ID                  XPAR_PSU_IPI_7_INT_ID
// #define IPI_8_INT_ID                  XPAR_PSU_IPI_8_INT_ID
// #define IPI_9_INT_ID                  XPAR_PSU_IPI_9_INT_ID
// #define IPI_10_INT_ID                 XPAR_PSU_IPI_10_INT_ID

/**
 * @brief Initiates the IPI driver
 * 
 * @param ipi pointer to the IPI driver instance
 * @param device_id device id of the IPI driver one wants to initiate
 * @return 0 if successful (int)
 */
int ipi_init(XIpiPsu * ipi, u16 device_id);


/**
 * @brief Sends a message to the receiver
 * 
 * @param ipi pointer to the IPI driver instance
 * @param receiver the receiver id
 * @param msg the message to send, 32 bit implemented. 32 bytes max
 * @return 0 if successful (int)
 */
int ipi_send(XIpiPsu * ipi, u32 receiver, u32 *msg, u32 msg_len);

/**
 * @brief Reads a message from the IPI
 * 
 * @param ipi pointer to the IPI driver instance
 * @param expected_source the expected source of the message
 * @param msg pointer to the adress to store the message
 * @return 0 if successful (int)
 */
int ipi_read(XIpiPsu * ipi, u32 expected_source, u32 *msg, u32 msg_len);


/**
 * @brief Checks the status of the IPI
 * 
 * @param ipi pointer to the IPI driver instance
 * @param receiver the receiver id
 * @return 0 if reciver is free (int)
 */
int obs_status(XIpiPsu *ipi, u32 receiver);

#endif /* SRC_DRIVERS_IPI_IPI_H_ */
