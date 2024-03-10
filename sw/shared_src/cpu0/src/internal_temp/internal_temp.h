/*
 * internal_temp.h
 *
 *  Created on: Jan 14, 2023
 *      Author: Eskil
 */

#ifndef SRC_DRIVERS_INTERNAL_TEMP_H_
#define SRC_DRIVERS_INTERNAL_TEMP_H_

/** 
 * @brief Initializes the internal temperature sensor.
 */
void internal_temp_init();

/**
 * @brief Reads the internal temperature sensor and returns the value.
 * @return The internal temperature in degrees Celsius. (u32)
 */
float internal_temp_get();

#endif /* SRC_DRIVERS_INTERNAL_TEMP_H_ */
