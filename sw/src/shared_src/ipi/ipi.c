#include "ipi.h"
#include "xil_types.h"
#include "xstatus.h"
#include "../../utility/log.h"
#include "../../utility/print_additions.h"


int obs_status(XIpiPsu *ipi, u32 cpu_id)
{
    u32 status;

    status = XIpiPsu_GetObsStatus(ipi);
    
    log_ipi("obs_status: ");

    #ifdef LOG_IPI
        print_binary(status);
    #endif

    if (status & cpu_id) {
        log_ipi("CPU %d is busy\r\n", cpu_id);
        return XST_FAILURE;
    }


    return XST_SUCCESS;
}


int ipi_init(XIpiPsu * ipi, u16 device_id)
{
    XIpiPsu_Config * cfg_ptr;
    u32 status;

    cfg_ptr = XIpiPsu_LookupConfig(device_id);
    if(NULL == cfg_ptr) {
        return XST_FAILURE;
    }

    status = XIpiPsu_CfgInitialize(ipi, cfg_ptr, cfg_ptr->BaseAddress);
    if(XST_SUCCESS != status) {
        return XST_FAILURE;
    }

    XIpiPsu_InterruptEnable(ipi, XIPIPSU_ALL_MASK);

    XIpiPsu_ClearInterruptStatus(ipi, XIPIPSU_ALL_MASK);

    return XST_SUCCESS;
}

int ipi_send(XIpiPsu * ipi, u32 receiver, u32 *msg, u32 msg_len)
{
    u32 status;

    status = obs_status(ipi, receiver);
    if (status != 0) {
        log_ipi("obs_status failed\r\n");
        return XST_FAILURE;
    }
    

    status = XIpiPsu_WriteMessage(ipi, receiver, msg, msg_len, XIPIPSU_BUF_TYPE_MSG);
    if(XST_SUCCESS != status) {
        return XST_FAILURE;
    }

    status = XIpiPsu_TriggerIpi(ipi, receiver);
    if(XST_SUCCESS != status) {
        return XST_FAILURE;
    }

    // Check if pollforack business needed

    return XST_SUCCESS;
}


int ipi_read(XIpiPsu *ipi, u32 expected_source, u32 *message, u32 msg_len) {
    u32 status;

    u32 source = XIpiPsu_GetInterruptStatus(ipi);

    if (!(source & expected_source)) {

        return XST_FAILURE; 
    }

    status = XIpiPsu_ReadMessage(ipi, expected_source, message, msg_len, XIPIPSU_BUF_TYPE_MSG);
    if (XST_SUCCESS != status) {

        return XST_FAILURE;
    }

    // Pru32 each element of the message
    for (u32 i = 0; i < msg_len; i++) {
        log_ipi("CPU0 - IPI message received: %d\r\n", message[i]);
    }

    XIpiPsu_ClearInterruptStatus(ipi, expected_source);

    return XST_SUCCESS;
}
