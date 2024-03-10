#include "conversions.h"

u64 swap_endian_64(u64 data) {
    union {
        u64 u64;
        u8  u8[8];
    } x;

    x.u64 = data;

    for (int i = 0; i < 4; i++) {
        u8 temp     = x.u8[i];
        x.u8[i]     = x.u8[7 - i];
        x.u8[7 - i] = temp;
    }
    return x.u64;
}

u32 swap_endian_32(u32 data) {
    return (u32)(swap_endian_64(data) >> 32);
}
