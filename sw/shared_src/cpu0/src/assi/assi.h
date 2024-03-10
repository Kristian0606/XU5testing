/*
 * assi.h
 *
 *  Created on: 4 Apr 2022
 *      Author: Audun Amadeus Olsen
 */

#ifndef SRC_ASSI_ASSI_H_
#define SRC_ASSI_ASSI_H_

typedef enum {
    AS_off,
    AS_ready,
    AS_driving,
    AS_emergency,
    AS_finished
} assi_state;

void handle_assi(assi_state state);

#endif /* SRC_ASSI_ASSI_H_ */
