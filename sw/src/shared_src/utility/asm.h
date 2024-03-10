//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/zcan/canfd.c
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 16-04-19

#ifndef SRC_UTILITY_ASM_H_
#define SRC_UTILITY_ASM_H_

#include "xil_io.h"

// #define CPU1_STACK_PTR 0xFFFFFFF0 /* Constant */
// #define CPU1_STACK_ADR 0x20080000 /* Defined by CPU1's linker script */

// static inline void wake_cpu1() {
//     /* Load CPU1 stack into CPU1 stack pointer */
//     Xil_Out32(CPU1_STACK_PTR, CPU1_STACK_ADR);
//     asm("DMB SY"); /* Wait for memory operations to complete */
//     asm("SEV"); /* Send Event to wake CPU1 */
// }

/**
 * @brief Enter critical section, disable local interrupts
 
 */
static inline void ENTER_CRITICAL() {
    asm("MSR DAIFSet, #2");
}


/**
 * @brief Leave critical section, enable local interrupts and wait for memory completion
 
 
 */
static inline void LEAVE_CRITICAL() {
    asm("MSR DAIFClr, #2");
    asm("ISB");
}

#endif /* SRC_UTILITY_ASM_H_ */
