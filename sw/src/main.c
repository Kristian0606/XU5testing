
#include    "xparameters.h"
#include    "shared_src/intc/intc.h"
#include    "shared_src/ipi/ipi.h"
#include    "xil_printf.h"
#include    "xil_types.h"
#include    "shared_src/utility/asm.h"
#include    "utility/pmu.h"
#include    "utility/print_additions.h"



static XScuGic          intc_i;
static XIpiPsu          ipi_i;
static u32              status;
static u32              uptime;



static u32 msg_len = 1;
static u32 ipi_to_send = 1000;
u32 ipi_received = 0;

ipi_message_t message;
u32* message_ptr = (u32*)&message;

void init_message() {
    for (int i = 0; i < msg_len; i++) {
        message.data[i] = 0x01010101 + i;
    }
}


static void tick_fit_isr(void* ref);
static void ipi_1_isr();


int main(){
    xil_printf("CPU0 - Starting main.\r\n");

    init_message();
    // init_message_maxbuffer();




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
    xil_printf("CPU0 - IPI 1 initialized.\r\n");



    /* Connect ISRs to IPI*/
    status = intc_connect_isr_to_ipi(&intc_i, IPI_APU0_INT_ID, ipi_1_isr, &ipi_i);
    if (status != XST_SUCCESS) {
        xil_printf("CPU0 - Failed to connect IPI ISR to interrupt controller 1.\r\n");
        return XST_FAILURE;
    }
    xil_printf("CPU0 - IPI ISR connected to interrupt controller 1.\r\n");


    // status = intc_connect_isr(&intc_i, IRQ_FIT,         tick_fit_isr);
    // if (status != XST_SUCCESS) {
    //     xil_printf("CPU0 - Failed to connect FIT ISR to interrupt controller.\r\n");
    //     return XST_FAILURE;
    // }
    // xil_printf("CPU0 - FIT ISR connected to interrupt controller.\r\n");

    intc_enable();
    
    for (u32 n = 0; n < 3; n++) {

        u32 start_time = pmu_get_counter();
        //start benchmark
        
        for (u32 i = 0; i < ipi_to_send; i++) {
            ipi_send(&ipi_i, IPI_APU0, message_ptr, msg_len);
        }
        //end benchmark

        u32 end_time = pmu_get_counter();
        u32 cycles = end_time - start_time;

        xil_printf("CPU0 - Cycles used run %d: %d\r\n", n, cycles);
        xil_printf("CPU0 - IPIs sent: %d\r\n", ipi_to_send);
        xil_printf("CPU0 - IPIs received: %d\r\n", ipi_received);

    }
    while(1){
    	asm("NOP");
    }



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

    // if (tick_counter % TICKS_1_Hz == 0) {
    //     uptime++;

    //     status = ipi_send(&ipi_i, IPI_APU0, message_ptr, msg_len);
    //     if (status != XST_SUCCESS) {
    //         xil_printf("CPU0 - Failed to send IPI message to APU0.\r\n");
    //     }
    //     xil_printf("CPU0 - IPI message sent to APU0.\r\n");

    // }

    ++tick_counter;
}


/**  
 * @brief This function is called when the IPI interrupt is triggered.
 *          It reads the IPI message and adds the corresponding event to the HSM.
 */
static void ipi_1_isr(void *ref){
    //xil_printf("CPU0 - IPI ISR 1: %d\r\n", ref);
    ipi_message_t ipi_msg;
    u32* ipi_ptr = (u32*)&ipi_msg;
    
    ipi_read(&ipi_i, IPI_APU0, ipi_ptr, msg_len);
    ipi_received++;

}
