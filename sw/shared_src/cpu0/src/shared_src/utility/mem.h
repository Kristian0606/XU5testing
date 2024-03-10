//   _____ _____ _____
//  |   | |__   |   | | cpu1/src/utility/mem.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 12-05-19
#ifndef SRC_UTILITY_MEM_H_
#define SRC_UTILITY_MEM_H_

#include "shared_types.h"

/*
    * Memory map:
    * Of shared memory:
    * Dynamic memory allocation so changes in the struct will affect the memory map.
    * 
*/
#define SHARED_ADR_START       (0x60000000) /* Start of shared memory */
#define SHARED_ADR_TV_INPUT    (SHARED_ADR_START) 
#define SHARED_ADR_TV_OUTPUT   (SHARED_ADR_START + sizeof(tv_input_t)) 
#define SHARED_ADR_TV_CONFIG   (SHARED_ADR_TV_OUTPUT + sizeof(tv_output_t)) 
#define SHARED_ADR_CPU1_STATUS (SHARED_ADR_TV_OUTPUT + sizeof(tv_config_t)) 
#define SHARED_ADR_END         (SHARED_ADR_CPU1_STATUS + sizeof(cpu_status_t)) /* End of shared memory */

// CPU0 definitions
static inline void mem_w_tv_input(tv_input_t * src_adr) {
    memcpy((u32 *)SHARED_ADR_TV_INPUT, (u32 *)src_adr, sizeof(tv_input_t));
    asm("DMB SY"); /* Wait for memory instructions to complete */
}

static inline void mem_r_tv_output(tv_output_t * dest_adr) {
    memcpy((u32 *)dest_adr, (u32 *)SHARED_ADR_TV_OUTPUT, sizeof(tv_output_t));
    asm("DMB SY"); /* Wait for memory instructions to complete */
}

static inline void mem_r_cpu1_status(cpu_status_t * dest_adr) {
    memcpy((u32 *)dest_adr, (u32 *)SHARED_ADR_CPU1_STATUS, sizeof(cpu_status_t));
    asm("DMB SY");
}

// CPU1 definitions
static inline void mem_r_tv_input(tv_input_t * dest_adr) {
    memcpy((u32 *)dest_adr, (u32 *)SHARED_ADR_TV_INPUT, sizeof(tv_input_t));
    asm("DMB SY"); /* Wait for memory instructions to complete */
}

static inline void mem_w_tv_output(tv_output_t * src_adr) {
    memcpy((u32 *)SHARED_ADR_TV_OUTPUT, (u32 *)src_adr, sizeof(tv_output_t));
    asm("DMB SY"); /* Wait for memory instructions to complete */
}

static inline void mem_w_cpu1_status(cpu_status_t * src_adr) {
    memcpy((u32 *)SHARED_ADR_CPU1_STATUS, (u32 *)src_adr, sizeof(cpu_status_t));
    asm("DMB SY");
}

// Shared definitions
static inline void mem_w_tv_config(tv_config_t * src_adr) {
    memcpy((u32 *)SHARED_ADR_TV_CONFIG, (u32 *)src_adr, sizeof(tv_config_t));
    asm("DMB SY"); /* Wait for memory instructions to complete */
}

static inline void mem_r_tv_config(tv_config_t * dest_adr) {
    memcpy((u32 *)dest_adr, (u32 *)SHARED_ADR_TV_CONFIG, sizeof(tv_config_t));
    asm("DMB SY");
}

#endif /* SRC_UTILITY_MEM_H_ */
