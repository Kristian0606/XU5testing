#include "xil_printf.h"
#include "xil_types.h"


void print_binary(u32 num) {
    for (s32 i = 31; i >= 0; i--) {
        u32 bit = (num >> i) & 1;
        xil_printf("%u", bit);
        
        if (i % 8 == 0) {
            xil_printf(" ");
        }
    }
    xil_printf("\r\n");
}