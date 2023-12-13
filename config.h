#include "pins.h"

//------------------------      SERIAL DISPLAY CONFIGURATION     --------------------------//

// This must be enabled for serial communication with the uC
// uC will not enable the serial terminal without 
#define ENABLE_SERIAL_OUTPUT true
#define SERIAL_BAUD 115200
#define SERIAL_DISPLAY_FREQ 4

//------------------------      CLIMATE CONTROL SETTINGS     --------------------------//

#define MIN_TARGET_TEMP 18      //  lowest internal temperature the CC system will try to reach
#define MAX_TARGET_TEMP 28      // highest internal temperature the CC system will try to reach


// this value should be greater than 5 to average out noisy readings
#define N_THERMISTOR_POLLS 20       // number of voltage readings to average for calculating the temperature
#define THERMISTOR_POLL_FREQ 60     // frequency at which CC system will take readings of temperature sensors

// frequency to poll digital and analogue inputs
#define SNSR_POLLING_FREQ 20

//------------------------      THERMISTOR INPUT SAFETY CONFIGURATION     --------------------------//
// WARNING: Enabled thermistors will be exposed to 5V in an unsafe way if these flags are set to true without the proper precautions 
//          
//          If disconnected, ensure that the pin is not pulled up to the 5V rail by disconnecting the 20k resistor or disabling the relevant flag
bool THERMISTOR_INPUT_POPULATED_FLAGS [N_THERMISTOR] = { 0 };

//------------------------      INPUT / OUTPUT PIN ASSIGNMENT     --------------------------//

// Temperature Control Potentiometer <- ANG_IN 1 
uint8_t TEMPCTL_POT_PIN = ANG_IN_PINS [ 0 ];



//------------------------      MISC SETTINGS   --------------------------//
#define ENABLE_ONBOARD_LED true