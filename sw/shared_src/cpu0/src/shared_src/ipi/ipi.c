/*
 * ipi.c
 *
 *  Created on: Nov 18, 2020
 *      Author: Eskil
 */

#include "ipi.h"

#include "xstatus.h"

int ipi_init(XIpiPsu * ipi, u16 device_id)
{
    XIpiPsu_Config * cfg_ptr;
    int status;

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

int ipi_send(XIpiPsu * ipi, u32 receiver, u32 msg)
{
    const int msg_len = 1;
    int status;

    status = XIpiPsu_WriteMessage(ipi, receiver, &msg, msg_len, XIPIPSU_BUF_TYPE_MSG);
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

int ipi_read(XIpiPsu * ipi, u32 expected_source, u32 * message)
{
    const int msg_len = 1;
    int status;

    u32 source = XIpiPsu_GetInterruptStatus(ipi);

    if(!(source & expected_source)) {
        return XST_FAILURE; // source and expected source not the same.
    }

    status = XIpiPsu_ReadMessage(ipi, expected_source, message, msg_len, XIPIPSU_BUF_TYPE_MSG);
    if(XST_SUCCESS != status) {
        return XST_FAILURE;
    }

    XIpiPsu_ClearInterruptStatus(ipi, expected_source);

    return XST_SUCCESS;
}
