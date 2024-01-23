/**
 * PIN ASSIGNMENTS
 * UniqueEVs Climate Control Board Rev. 2
*/

#ifndef PINS_H
#define PINS_H

// Define pins that should be disabled on boot as they aren't used
/// @warning DO NOT CHANGE ASSIGNMENTS
#define N_UNUSED_PINS 2
uint8_t PINS_UNUSED [N_UNUSED_PINS] = { 25, 32 }; 

#define PIN_ONBOARD_LED 13

//------------------------      INPUTS     --------------------------//

// number of CAN transceivers
#define N_CAN 3

// DIGITAL INPUTS
#define N_DIG_IN 8
uint8_t PINS_DIG_IN [N_DIG_IN] = { 2, 3, 4, 5, 6, 7, 8, 9 };

// ANALOGUE INPUTS         
#define N_ANG_IN 4
uint8_t PINS_ANG_IN [N_ANG_IN] = { A16, A17, A14, A15 };

// NTC 10K THERMISTOR INPUTS       
#define N_THERMISTOR 6
uint8_t PINS_THERMISTOR [N_THERMISTOR] = { A0, A1, A2, A3, A4, A5 };

//------------------------      OUTPUTS     --------------------------//

// DIGITAL OUTPUTS
#define N_DIG_OUT 4
uint8_t PINS_DIG_OUT [N_DIG_OUT] = { 24, 12, 11, 10 };

// RELAY OUTPUTS               
#define N_RELAY 5
uint8_t PINS_RELAY [N_RELAY] = { 26, 27, 35, 34, 33 };

// PWM SIGNAL      
#define N_SIG_PWM 2
uint8_t PINS_SIG_PWM [N_SIG_PWM] = { 28, 29 };

// PWM POWER
#define N_PWR_PWM 2
uint8_t PINS_PWR_PWM_EN [N_PWR_PWM] = { 20, 21 };   // enable pins
uint8_t PINS_PWR_PWM_S  [N_PWR_PWM] = { 36, 37 };   // driver signal pins

// Transceivers are matched to MCU interfaces
// CAN 1 -> RX/TX 1
// CAN 2 -> RX/TX 2
// CAN 3 -> RX/TX 3

#endif 