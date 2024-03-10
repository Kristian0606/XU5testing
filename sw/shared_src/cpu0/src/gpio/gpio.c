//  GPIO Configuration
//  Made by: Andreas Larsen
//
//  GPIO Vector defined in I/O planning in synthesized design stage in Vivado,
//  with a number of bits down to 0. These are mapped to specific pins and
//  listed below, after definition.
//
//

#include "gpio.h"
#include "xgpiops.h"

#define DIRECTION_INPUT  0U
#define DIRECTION_OUTPUT 1U

static XGpioPs gpio;

void gpio_init() {

    // Initialize GPIO
    XGpioPs_Config * p_gpio_cfg = XGpioPs_LookupConfig(XPAR_PSU_GPIO_0_DEVICE_ID);
    XGpioPs_CfgInitialize(&gpio, p_gpio_cfg, p_gpio_cfg->BaseAddr);

    // Initialize MIO Outputs
    XGpioPs_SetDirectionPin(&gpio, PS_DEBUG, DIRECTION_OUTPUT); // output set
    XGpioPs_SetOutputEnablePin(&gpio, PS_DEBUG, 1U);            // controlling output enabled
    XGpioPs_WritePin(&gpio, PS_DEBUG, 1);

    // Initialize EMIO Outputs
    XGpioPs_SetDirectionPin(&gpio, UART0_DE, DIRECTION_OUTPUT); // output set
    XGpioPs_SetDirectionPin(&gpio, ACTIVATE_LATCH, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, WATCHDOG_OUT, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, ACTIVATE_TS, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, EBS_ENGAGE, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, ASB_ENGAGE, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, ASSI_YELLOW, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, ASSI_BLUE, DIRECTION_OUTPUT);
    XGpioPs_SetDirectionPin(&gpio, SYNC_OUT, DIRECTION_OUTPUT);

    XGpioPs_SetOutputEnablePin(&gpio, UART0_DE, 1U); // controlling output enabled
    XGpioPs_SetOutputEnablePin(&gpio, ACTIVATE_LATCH, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, WATCHDOG_OUT, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, ACTIVATE_TS, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, EBS_ENGAGE, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, ASB_ENGAGE, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, ASSI_YELLOW, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, ASSI_BLUE, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, SYNC_OUT, 1U);

    XGpioPs_WritePin(&gpio, UART0_DE, 1);
    XGpioPs_WritePin(&gpio, ACTIVATE_LATCH, 0);
    XGpioPs_WritePin(&gpio, WATCHDOG_OUT, 0);
    XGpioPs_WritePin(&gpio, ACTIVATE_TS, 0);
    XGpioPs_WritePin(&gpio, EBS_ENGAGE, 0);
    XGpioPs_WritePin(&gpio, ASB_ENGAGE, 0);
    XGpioPs_WritePin(&gpio, ASSI_YELLOW, 0);
    XGpioPs_WritePin(&gpio, ASSI_BLUE, 0);
    XGpioPs_WritePin(&gpio, SYNC_OUT, 0);

    // Initialize EMIO Inputs
    XGpioPs_SetDirectionPin(&gpio, GPS_PPS, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, SDC_STATUS, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, WATCHDOG_STATUS, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, LATCH_STATUS, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, ASMS_ON, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, ASMS_ACTUALLY_ON, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, EBS_STATUS, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, ASB_STATUS, DIRECTION_INPUT);
    XGpioPs_SetDirectionPin(&gpio, SYNC_IN, DIRECTION_INPUT);
}

void write_gpio_pin(gpio_pin_t pin, u8 value) {
    XGpioPs_WritePin(&gpio, (u32)pin, value);
}

void toggle_gpio_pin(gpio_pin_t pin) {
    u32 pin_state = XGpioPs_ReadPin(&gpio, (u32)pin);
    XGpioPs_WritePin(&gpio, (u32)pin, !pin_state);
}

u32 read_gpio_pin(gpio_pin_t pin) {
    return XGpioPs_ReadPin(&gpio, (u32)pin);
}
