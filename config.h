/** 
 * CONFIGURATION FILE
 * UniqueEVs Climate Control board Rev. 2
 * 
 * This file is used as the assignments for inputs and outputs defined in pins.h
 * This file also contains the setpoints for dynamic system behaviour and control 
 * 
 * Example: 
 * THERM_1 is going to be used as reservoir temp:
 * In this file define that mapping. i.e:
 * #define TEMP_SNSR_RESERVOIR PINS_THERMISTOR [0] 
*/

#ifndef CONFIG_H
#define CONFIG_H

#define CC_SOFTWARE_VERSION 0.1

#include "pins.h"
#include "TempRange.h"

//------------------------      CIRCUIT TUNING      ---------------------------------------//

#define ANG_IN_MAX_RES 875

//------------------------      SERIAL DISPLAY CONFIGURATION     --------------------------//

#define SERIAL_INIT_TIMEOUT_MS 5000
#define DEFAULT_SERIAL_BAUD 9600
#define SERIAL_BAUD 115200

// This must be enabled for serial communication with the uC
// uC will not enable the serial terminal without 
#define ENABLE_SERIAL_OUTPUT true
#define SERIAL_DISPLAY_FREQ 3.0f
#define SERIAL_MSG_BUFFER_SIZE 1024

// SERIAL DISPLAY FLAGS
#define DISPLAY_TEMPERATURES    false
#define DISPLAY_DIG_INPUTS      false
#define DISPLAY_ANG_INPUTS      false

//------------------------      TEMPERATURE SAFETY SETTINGS     --------------------------//

/// @brief temperature range for batteries
#define SAFETY_TEMP_BATT_MIN_OK -5.0f
#define SAFETY_TEMP_BATT_MAX_OK 50.0f
#define SAFETY_TEMP_BATT_MIN_CRIT 65.0f
#define SAFETY_TEMP_BATT_MAX_CRIT 150.0f

/// @brief temperature range for charger
#define SAFETY_TEMP_CHGR_MIN_OK -5.0f
#define SAFETY_TEMP_CHGR_MAX_OK 50.0f
#define SAFETY_TEMP_CHGR_MIN_CRIT 65.0f
#define SAFETY_TEMP_CHGR_MAX_CRIT 150.0f

/// @brief temperature range for main reservoir
#define SAFETY_TEMP_RES_MIN_OK -5.0f
#define SAFETY_TEMP_RES_MAX_OK 50.0f
#define SAFETY_TEMP_RES_MIN_CRIT 65.0f
#define SAFETY_TEMP_RES_MAX_CRIT 150.0f

//------------------------      CLIMATE CONTROL SETTINGS     --------------------------//

#define MIN_TARGET_TEMP 18      //  lowest internal temperature the CC system will try to reach
#define MAX_TARGET_TEMP 28      // highest internal temperature the CC system will try to reach

// number of voltage readings to average for calculating the temperature
// this value should be greater than 5 to average out noisy readings
#define THERMISTOR_READING_BUFFER_SIZE 20       

// frequency to take readings of temperature sensors
#define THERMISTOR_POLL_FREQ 100 // Hz

// frequency to poll digital and analogue inputs
#define SNSR_POLLING_FREQ 75 // Hz

//------------------------      THERMISTOR INPUT SAFETY CONFIGURATION     --------------------------//

/** @warning DO NOT CHANGE */
#define THERMISTOR_VOLTAGE_DIV_RESISTANCE 20000.0f

/**
 * @warning Enabled thermistors will be exposed to 5V in an unsafe way if these flags are set to true without the proper precautions
 *  
 *  (5V) ----[20k]---(*)---[NTC10k]--- (GND)
 *                    |
 *                  (ADC)
 * 
 * If disconnected, ensure that the pin is not pulled up to the 5V rail by desoldering the 20k resistor.
 * ONLY add the resistor if the input is going to be used! otherwise the 5V will have no path to ground 
*/
bool FLAG_THERMISTOR_POPULATED [N_THERMISTOR] = { 
    false, // THERM 1 
    false, // THERM 2 
    false, // THERM 3 
    false, // THERM 4 
    false, // THERM 5 
    false  // THERM 6 
};

//------------------------      INPUT / OUTPUT PIN ASSIGNMENT     --------------------------//

/** 
 * @warning Flags must be configured or inputs will be disabled
*/
bool FLAG_DIG_IN_POPULATED [N_DIG_IN] = { 
    true,  // DIG 1
    true,  // DIG 2
    true,  // DIG 3
    true,  // DIG 4
    false,  // DIG 5
    false,  // DIG 6
    false,  // DIG 7
    false,  // DIG 8
};

/** 
 * @warning Flags must be configured or inputs will be disabled
*/
bool FLAG_ANG_IN_POPULATED [N_ANG_IN] = { 
    false,  // ANG 1
    false,  // ANG 2
    false,  // ANG 3
    false   // ANG 4
};

/**
 * @warning Flags must be configured or CAN transceivers will not be initialised
*/
bool FLAG_CAN_POPULATED [N_CAN] = {
    false,  // CAN 1
    false,  // CAN 2
    false   // CAN 3
};

//------------------------      PWM    --------------------------//

// pwm will default to 1 kHz
#define DEFAULT_PWM_FREQ 1000.0

#define PWR_PWM_FREQ DEFAULT_PWM_FREQ
#define SIG_PWM_FREQ DEFAULT_PWM_FREQ

/// @brief SIG PWM outputs are inverted. i.e duty cycle of 80% will actually output 20%
// if unset, the SIG PWM lines will be inverted when using setSIGPWM ()
#define INVERT_SIG_PWM true

//------------------------      MISC SETTINGS   --------------------------//


// tune this in accordance to SNSR_POLLING_FREQ 
// See https://www.etlcpp.com/debounce.html for details
#define DEBOUNCE_VALID 5
#define DEBOUNCE_HOLD 5

/// @brief set the LED to be always on
#define ONBOARD_LED_ENABLE_STATIC true
/// @brief set the LED to be on by default and off while taking a temperature or sensor sample
#define ONBOARD_LED_ENABLE_SAMPLE true

#endif