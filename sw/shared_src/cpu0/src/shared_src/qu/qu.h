//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/hsm/qu.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 24-04-19
#ifndef SRC_HSM_QU_H_
#define SRC_HSM_QU_H_

#include "xil_types.h"
#include "../utility/asm.h"

// This queue relies on the fact that u8 overflows after 255. See
// https://github.com/NinjaNymo/C_StateMachines/blob/master/README.md#the-queue
// for an explanation.

typedef struct {
    u8 fifo[256];
    u8 head;
    u8 tail;
} qu_t;

static inline void STANDBY() {
    asm("ISB");
    asm("WFI");
}

static inline void enqu_isr(volatile qu_t * qu, u8 in) {
    qu->fifo[qu->tail++] = in;
}

static inline void enqu(volatile qu_t * qu, u8 in) {
    ENTER_CRITICAL();
    qu->fifo[qu->tail++] = in;
    LEAVE_CRITICAL();
}

static inline void dequ(volatile qu_t * qu, u8 * out) {
    while (qu->head == qu->tail) {
        STANDBY();
    }

    ENTER_CRITICAL();
    *out = qu->fifo[qu->head++];
    LEAVE_CRITICAL();
}

static inline u8 qulvl(volatile qu_t * evqu) {
    ENTER_CRITICAL();
    u8 head = evqu->head;
    u8 tail = evqu->tail;
    LEAVE_CRITICAL();

    u8 lvl;
    if (head == tail) {
        lvl = 0;
    } else if (head < tail) {
        lvl = tail - head;
    } else {
        lvl = (255 - head) + tail + 1;
    }
    return lvl;
}

#endif /* SRC_HSM_QU_H_ */
