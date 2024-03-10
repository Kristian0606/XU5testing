/*
 * actuators.h
 *
 *  Created on: 2 Apr 2022
 *      Author: DANKLOARD
 */

#ifndef SRC_ACTUATORS_ACTUATORS_H_
#define SRC_ACTUATORS_ACTUATORS_H_

#include "stdbool.h"
#include "../utility/types.h"

bool actuators_solenoid_ok(void);
void actuators_activate_ebs(void);
void actuators_deactivate_ebs(void);
bool actuators_is_ebs_engaged();
bool actuators_is_hydraulic_brakes_engaged(vcu_data_t * vcu_data_p);

#endif /* SRC_ACTUATORS_ACTUATORS_H_ */
