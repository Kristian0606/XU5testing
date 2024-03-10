/**
 * @file sensor_errorfield.h
 * @author Sivaranjith Sivarasa
 * @brief Functions for checking if the sensor is in a OK state, defined by the SBS
 * @version 0.1
 * @date 2021-05-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef SRC_SAFETY_SENSOR_ERRORFIELD_H_
#define SRC_SAFETY_SENSOR_ERRORFIELD_H_

#include "xil_types.h"

/**
 * @brief Given a error bitfield and a error offset for the sensor in the bitfielded,
 * check there state is ok. This function will return true if ok and false if not ok.
 *
 * @param error_bf The errorbitfield
 * @param error_offset The offset for the sensor
 * @return u8 return 1 if sensor is not ok, 0 otherwise
 */
u8 sensor_errorfield_ok(u32 error_bf, u8 error_offset);

#endif /* SRC_SAFETY_SENSOR_ERRORFIELD_H_ */
