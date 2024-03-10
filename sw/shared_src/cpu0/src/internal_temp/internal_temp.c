#include "internal_temp.h"

#include "xsysmonpsu.h"
#include "xparameters.h"

static XSysMonPsu sysmon;

void internal_temp_init() {    
    XSysMonPsu_Config * config_ptr = XSysMonPsu_LookupConfig(XPAR_XSYSMONPSU_0_DEVICE_ID);
    XSysMonPsu_CfgInitialize(&sysmon, config_ptr, config_ptr->BaseAddress);
    XSysMonPsu_SetSequencerMode(&sysmon, XSM_SEQ_MODE_SAFE, XSYSMON_PS);
    XSysMonPsu_SetAvg(&sysmon, XSM_AVG_16_SAMPLES, XSYSMON_PS);
}

float internal_temp_get() {
    u32 temperature_reading = XSysMonPsu_GetAdcData(&sysmon, XSM_CH_TEMP, XSYSMON_PS);
    return XSysMonPsu_RawToTemperature_OnChip(temperature_reading);
}