//  GPIO Configuration
//  Made by: Andreas Larsen
//
//  GPIO Vector defined in I/O planning in synthesized design stage in Vivado,
//  with a number of bits down to 0. These are mapped to specific pins and
//  listed below, after definition.
//
//

#ifndef SRC_GPIO_GPIO_H_
#define SRC_GPIO_GPIO_H_

#include "xil_types.h"
typedef enum {
    // MIO outputs
    PS_DEBUG = 51U,
    // EMIO Inputs         GPIO     Pin ID     Use
    SYNC_IN = 54U,    // R17
    ASB_STATUS,       // W18     Status of secondary emergency brake switch.
    EBS_STATUS,       // U19     Status of primary emergency brake switch.
    ASMS_ACTUALLY_ON, // AA20    Status of Autonomous Systems Master Switch.
    ASMS_ON,          // AB16    Status of Autonomous Systems Master Switch.
    LATCH_STATUS,     // AB17    Status of the emergency brake switch latch.
    WATCHDOG_STATUS,  // U2      Status of the watchdog.
    SDC_STATUS,       // F1      Status of shutdown circtuit.
    GPS_PPS,          // AA19    GPS Pulse Per Second. Not used right now, but it is automatically generated from VN-310 and can be used with if wanted.

    //  EMIO Outputs
    SYNC_OUT,       // AB19
    ASSI_BLUE,      // Y18     Signal to activate Yellow ASSI. Lights as long as signal is active.
    ASSI_YELLOW,    // Y19     Signal to activate Yellow ASSI. Lights as long as signal is active.
    ASB_ENGAGE,     // V16     Redundancy for EBS signal. Should be set to mirror EBS_engage.
    EBS_ENGAGE,     // W16     Active low signal to trigger release of emergency brake switch.
    ACTIVATE_TS,    // U1      Signal to allow TS activation. Low signal will open shutdown circuit.
    WATCHDOG_OUT,   // F2      Alive tick to keep watchdog from opening shutdown circuit.
    ACTIVATE_LATCH, // V18     Send low to trigger shutdown circuit latch.
    UART0_DE,       // AB22    Drive enable for RS422.
} gpio_pin_t;

// Global functions

/**
 * @brief Initializes the GPIO pins to desired functionality.
*/
void gpio_init();                              

/**
 * @brief Sets an output pin to desired value, 1 or 0
 * @param pin The pin to be set
 * @param value The value to be set, 1 or 0
*/
void write_gpio_pin(gpio_pin_t pin, u8 value); 

/** 
 * @brief Toggles an output pins value
 * @param pin The pin to be toggled
*/
void toggle_gpio_pin(gpio_pin_t pin);          

/** 
 * @brief Reads the logic level of an input pin
 * @param pin The pin to be read
 * @return The logic level of the pin, 1 or 0 (u32)
*/
u32  read_gpio_pin(gpio_pin_t pin);            // Returns 1 or 0 bares on input pin logic level

#endif /* SRC_GPIO_GPIO_H_ */
