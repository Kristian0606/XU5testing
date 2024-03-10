/*
 * can_types.h
 *
 * Created: 08.02.2016 18.15.19
 *  Author: Simen
 */

#ifndef CAN_TYPES_H_
#define CAN_TYPES_H_

#include "stdbool.h"
#include <stdint.h>

typedef union {
    // INTEGERS
    uint64_t u64;
    int64_t  i64;
    uint32_t u32[2];
    int32_t  i32[2];
    uint16_t u16[4];
    int16_t  i16[4];
    uint8_t  u8[8];
    int8_t   i8[8];

    // FLOAT
    float  f[2];
    double db;
} Can_data_t;

typedef enum {
    STANDARD_CAN_ID,
    EXTENDED_CAN_ID
} Can_ID_Type_t;

typedef struct {
    Can_data_t    data;
    uint8_t       dataLength;
    uint32_t      messageID;
    Can_ID_Type_t idType;
} Can_message_t;

#endif /* CAN_TYPES_H_ */
