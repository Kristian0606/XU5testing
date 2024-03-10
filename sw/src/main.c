
#include            "xparameters.h"
#include   "shared_src/intc/intc.h"
#include "shared_src/ipi/ipi.h"


static XScuGic          intc_i;
static XIpiPsu          ipi_i;
static u32              status;


static void tick_fit_isr(void* ref);
static void ipi_isr();


int main(){
    xil_printf("CPU0 - Starting main.\r\n");


    /* Initialize the hardware */
    status = intc_init(&intc_i);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to initialize interrupt controller.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - Interrupt controller initialized.\r\n");

    status = ipi_init(&ipi_i, IPI_APU0_ID);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to initialize IPI.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - IPI initialized.\r\n");


    /* Connect ISRs to IPI*/
    status = intc_connect_isr_to_ipi(&intc_i, IPI_APU0_INT_ID, ipi_isr, &ipi_i);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to connect IPI ISR to interrupt controller 1.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - IPI ISR connected to interrupt controller 1.\r\n");

    status = intc_connect_isr_to_ipi(&intc_i, IPI_APU1_INT_ID, ipi_isr, &ipi_i);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to connect IPI ISR to interrupt controller 7.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - IPI ISR connected to interrupt controller 7.\r\n");


    status = intc_connect_isr(&intc_i, IRQ_FIT,         tick_fit_isr);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to connect FIT ISR to interrupt controller.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - FIT ISR connected to interrupt controller.\r\n");


    status = intc_enable();
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to enable interrupt controller.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - Interrupt controller enabled.\r\n");

    


}




/**
 * @brief This function is called when the timer interrupt is triggered.
 *          Enabling the HSM to handle things at a specific frequency.
 * 
 * @param ref 
 */
static void tick_fit_isr(void* ref){
    static u32 tick_counter = 1;

    const u32 TICKS_PER_SEC = 10000;

    const u32 TICKS_1_Hz    = TICKS_PER_SEC /    1;
    const u32 TICKS_2_Hz    = TICKS_PER_SEC /    2;
    const u32 TICKS_5_Hz    = TICKS_PER_SEC /    5;
    const u32 TICKS_10_Hz   = TICKS_PER_SEC /   10;
    const u32 TICKS_20_Hz   = TICKS_PER_SEC /   20;
    const u32 TICKS_100_Hz  = TICKS_PER_SEC /  100;
    const u32 TICKS_1000_Hz = TICKS_PER_SEC / 1000;

    if (tick_counter % TICKS_1_Hz == 0) {
        uptime++;
        status = ipi_send(&ipi_i, IPI_APU1, 1);
        if (status != XST_SUCCESS) {
            xil_printf("CPU0 - Failed to send IPI message to APU1.\r\n");
        }
        xil_printf("CPU0 - IPI message sent to APU1.\r\n");


        status = ipi_send(&ipi_i, IPI_APU0, 10);
        if (status != XST_SUCCESS) {
            xil_printf("CPU0 - Failed to send IPI message to APU0.\r\n");
        }
        xil_printf("CPU0 - IPI message sent to APU0.\r\n");
    }


    ++tick_counter;
}


/**  
 * @brief This function is called when the IPI interrupt is triggered.
 *          It reads the IPI message and adds the corresponding event to the HSM.
 */
static void ipi_isr(){
    u32 ipi_msg;
    u32 status;
    status = ipi_read(&ipi_i, IPI_APU1, &ipi_msg);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to read IPI message from APU1.\r\n");
        return;
    }
    xil_printf("CPU0 - IPI message from APU1: %d.\r\n", ipi_msg);
    

    status = ipi_read(&ipi_i, IPI_APU0, &ipi_msg);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to read IPI message from APU0.\r\n");
        return;
    }
    xil_printf("CPU0 - IPI message from APU0: %d.\r\n", ipi_msg);
}
