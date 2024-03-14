//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/intc.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 14-01-19
#ifndef SRC_INTC_INTC_H_
#define SRC_INTC_INTC_H_

#include "xil_types.h"
#include "xscugic.h"
#include "xparameters.h"
#include "xipipsu.h"

// CPU's
#define CPU0_ID 1
#define CPU1_ID 2

// CPU0 IRQ's
#define IRQ_TIMER                    XPAR_SCUTIMER_INTR
#define IRQ_FIT                      XPAR_FABRIC_FIT_10KHZ_INTERRUPT_INTR
#define EPOS_IRQ                     XPAR_XTTCPS_0_INTR
#define INITIAL_CHECKUP_SEQUENCE_IRQ XPAR_XTTCPS_1_INTR
#define TTC_TIMER_3_IRQ              XPAR_XTTCPS_2_INTR

// CPU1 IRQ's
#define IRQ_FIT400                   XPAR_FABRIC_FIT_400HZ_INTERRUPT_INTR

typedef enum {
    SWIRQ_TV_INIT = 1,
    SWIRQ_TV_CONFIGURE,
    SWIRQ_SYNC_INPUT,
    SWIRQ_SYNC_OUTPUT,
    SWIRQ_SYNC_STATUS,
    SWIRQ_IDLE,
    SWIRQ_TV_TOGGLE_LC,
    SWIRQ_TV_TOGGLE_SKID,
    SWIRQ_TV_TOGGLE_STATIC,
    SWIRQ_TV_ENABLE_DV,
    SWIRQ_TV_DISABLE_DV,
    // Max 16 of these on the Z7000
} swirq_t;



/**
 * @brief Initializes the interrupt controller.
 * 
 * @param intc_p pointer to the interrupt controller instance
 * @return 0 if successful (int)
 */
int intc_init(XScuGic * intc_p);

/**
 * @brief Enables the interrupt controller.
 * 
 * @return 0 if successful (int)
 */
void intc_enable();

/**
 * @brief Connects an interupt service routine (ISR) to an Interrupt Request (IRQ).
 * 
 * @param intc_p pointer to the interrupt controller instance
 * @param irq_id the id of the interrupt request
 * @param isr pointer to the interrupt service routine
 * @return 0 if successful (int)
 */
int intc_connect_isr(XScuGic * intc_p, u32 irq_id, void * isr);

/**
 * @brief Connects an interupt service routine (ISR) to an Inter Processor Interrupt (IPI).
 * 
 * @param intc_p pointer to the interrupt controller instance
 * @param irq_id the id of the interrupt request
 * @param isr pointer to the interrupt service routine
 * @param ipi_p pointer to the IPI instance
 * @return 0 if successful (int)
 */
int intc_connect_isr_to_ipi(XScuGic * intc_p, u32 irq_id, void * isr, XIpiPsu *ipi_p);

#endif /* SRC_INTC_INTC_H_ */
