/**
 * @file sensor_errorfield.c
 * @author Sivaranjith Sivarasa
 * @brief Functions for checking if the sensor is in a OK state, defined by the SBS
 * @version 0.1
 * @date 2021-05-15
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "sensor_errorfield.h"
#include "sensors/SensorStatus_1_0.h"
#include "sensors/PedalPositions_1_0.h"

u8 sensor_errorfield_ok(u32 error_bf, u8 error_offset) {
    return ((error_bf >> error_offset) & sensors_PedalPositions_1_0_ERROR_BITMASK) == sensors_SensorStatus_1_0_STATUS_OK;
}
