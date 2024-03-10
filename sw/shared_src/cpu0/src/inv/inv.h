//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/inv/inv.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 04-06-19
#ifndef SRC_INV_INV_H_
#define SRC_INV_INV_H_
#include "../utility/types.h"

void inv_ctrl_static(vcu_data_t * vcu_data_p);
void inv_ctrl_static_speed(vcu_data_t * vcu_data_p);
/**
 * @brief Linearly map torque setpoints to the inverters still alive. The dead inverters will have zero Nm torque
 *
 * @param vcu_data_p pointer to the vcu_data_t struct containing inverter status
 */
void inv_ctrl_static_limp(vcu_data_t * vcu_data_p);
void inv_ctrl_zero(vcu_data_t * vcu_data_p);
void inv_ctrl_tv(vcu_data_t * vcu_data_p);
void inv_ctrl_enable_fw(vcu_data_t * vcu_data_p);
void inv_ctrl_enable(vcu_data_t * vcu_data_p);
void inv_ctrl_disable(vcu_data_t * vcu_data_p);

#endif /* SRC_INV_INV_H_ */
