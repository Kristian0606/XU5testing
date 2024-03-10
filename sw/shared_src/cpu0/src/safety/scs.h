//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/safety/scs.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 21-06-19
#ifndef SRC_SAFETY_SCS_H_
#define SRC_SAFETY_SCS_H_

#include "../utility/types.h"

void         scs_init(scs_t * scs);
scs_status_t scs_watchdog(scs_t * scs);
f32          scs_validate(f32 val, scs_t * scs, scs_status_t * status);
scs_status_t scs_validate_pu_setpoints(vcu_data_t * vcu_data_p);
scs_status_t scs_validate_tv(vcu_data_t * vcu_data_p);

#endif /* SRC_SAFETY_SCS_H_ */
