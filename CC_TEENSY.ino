/*
Teensy 4.1 based Automotive Climate Control System
Author: Brodie Collinson Davison (github: wombat02)

Copyright (c) 2024 UniqueEVs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Embedded_Template_Library.h>

// Interrupts
#include <avr/io.h>
#include <avr/interrupt.h>

// Teensy_PWM by Khoi Hoang
#include <Teensy_PWM.h>

#include "pins.h"
#include "config.h"
#include "orion_temp_sensor.h"
#include "canbus.h"
#include "fsm.h"

//----------------------------    [  ONBOARD CONFIG VARS  ]   ----------------------------//

// SERIAL DISPLAY
uint16_t _serial_display_delay_ms;              // time between sending serial messages (calculated at startup)
uint32_t _next_serial_display_time_ms;          // time at which the next serial message is sent
char* _serial_msg_bfr;                          // assign buffer on heap so that it isn't being re-allocated on the stack every call to serialDisplay ()

// NTC 10K Sensors
float* _temps;                             // actual temperature measurements
float* _temp_readings;                     // buffer to calculate averages of temp sensor readings  
uint8_t _n_accrued_temp_readings;          // number of readings placed into the readings buffer

// Input buffers
uint8_t*     _dig_readings;
uint16_t* _ang_readings;

// flags are set by the interval timers: flag indicates that the inputs should be polled and must be reset manually
volatile bool _flag_poll_thermistor;
volatile bool _flag_poll_snsr;
IntervalTimer _thermistor_poll_timer;
IntervalTimer _snsr_poll_timer;

// CAN transceivers
FlexCAN_T4 <CAN1, RX_SIZE_256, TX_SIZE_16> _can1;
FlexCAN_T4 <CAN2, RX_SIZE_256, TX_SIZE_16> _can2;
FlexCAN_T4 <CAN3, RX_SIZE_256, TX_SIZE_16> _can3;

// PWM 
float  _pwr_pwm_freq;
float  _sig_pwm_freq;
float* _sig_pwm_duty;
float* _pwr_pwm_duty;
bool*  _pwr_pwm_enabled;

/// @warning Both power PWM pins share a timer and both signal PWM pins share a timer. Meaning all power pwm will run at the same freq, likewise with the signal PWM.
// Hardware PWM library instances ( one for each pin )
Teensy_PWM* _hwpwm_pwr [N_PWR_PWM];
Teensy_PWM* _hwpwm_sig [N_SIG_PWM];

//----------------------------      CLIMATE CONTROL VARS   ----------------------------//

// temp ranges
/// @todo assign this memory as const and declare at compile time -> remove initTempRanges ()
TempRange* _temp_range_batt;
TempRange* _temp_range_res;
TempRange* _temp_range_chgr;


//---------------------------               FSM         --------------------------------//


// state instances and state list containing every state
sIdle      _sIdle;
sON        _sON;
sAuto      _sAuto;
sCharging  _sCharging;
sTempFault _sTempFault;

etl::ifsm_state* stateList [StateID::N_STATES] = {
    &_sIdle, &_sON, &_sAuto, &_sCharging, &_sTempFault
};

CCFSM fsm;

//----------------------------      FUNCTION DECLARATIONS   ----------------------------//

void setupOnboardLED      ();
void disableUnusedPins    ();
void setupSensors         ();
void setupTempSensors     ();
void setupCAN             ();
void setupPWM             ();
void setupPollingTimers   (); 
void initTempRanges       ();

void pollTempSensorsISR   ();
void pollSensorsISR       ();

void handlePollingMethods     ();
void handleSensorPolling      ();
void handleTempSensorPolling  ();

void pollTempSensors    ();
void pollDigitalInputs  ();
void pollAnalogueInputs ();

void setSIGPWM ( const uint8_t& pin, const float& freq, const float& duty );
void setPWRPWM ( const uint8_t& pin, const float& freq, const float& duty );

void serialDisplayInit ();
void serialDisplayConfigStatus ( int len, bool* flags );
void serialDisplay     ();



//----------------------------      PROGRAM SETUP   ----------------------------//

void setup() {

    /// @todo remove and assign memory at compile time
    // assign non I/O globals
    initTempRanges ();

    // disable all pins that aren't being used and configure onboard LED
    disableUnusedPins ();
    setupOnboardLED   ();

    // init sensors
    setupSensors     ();
    setupTempSensors ();

    // setup PMM 
    setupPWM ();

    // start polling timers
    setupPollingTimers ();

    // start CAN transceivers
    setupCAN ();


    // enable serial comms and initialise msg buffer / timing variables
    if ( ENABLE_SERIAL_OUTPUT ) {

        // assign message buffer on heap
        _serial_msg_bfr = new char [SERIAL_MSG_BUFFER_SIZE];

        _serial_display_delay_ms = 1000 / SERIAL_DISPLAY_FREQ;
        _next_serial_display_time_ms = 0;
        
        // start the serial port and wait for it to become active OR until the timeout duration
        Serial.begin ( SERIAL_BAUD );
        while ( !Serial && millis () <= SERIAL_INIT_TIMEOUT_MS );
        serialDisplayInit ();
    }

    // FSM
    fsm.set_states ( stateList, StateID::N_STATES );
    fsm.start ();

}// setup ()

//----------------------------      PROGRAM LOOP   ----------------------------//

void loop() {

    // get the current millisecond
    uint32_t cur_millis = millis ();

    // check polling flags and manage ISR flags
    // calls polling methods as required by the flags
    handlePollingMethods ();

    // SERIAL DISPLAY
    if ( ENABLE_SERIAL_OUTPUT ) {

        // serial display
        if ( cur_millis >= _next_serial_display_time_ms ) {
            
            // update next display time
            _next_serial_display_time_ms = cur_millis + _serial_display_delay_ms;
            
            // display serial message
            serialDisplay ();
        }
    }
}// loop ()

//----------------------------   [   FUNCTION DEFINITIONS    ]   ----------------------------//


//---------------------------   [   SETUP   ]  ----------------------------------//

/**
 * @fn setupOnboardLED ()
 * @brief set state of the ONBOARD LED based on configuration
*/
void setupOnboardLED ()
{
    // disable pin if not used
    if ( !ONBOARD_LED_ENABLE_STATIC && !ONBOARD_LED_ENABLE_SAMPLE )
        pinMode ( PIN_ONBOARD_LED, INPUT_DISABLE );
    else
        pinMode ( PIN_ONBOARD_LED, OUTPUT ); 


    // set onboard LED ON to indicate uC is running
    if ( ONBOARD_LED_ENABLE_STATIC ) { 
        digitalWrite ( PIN_ONBOARD_LED, HIGH ); 
    } 
}// setupOnboardLED ()

/**
 * @fn disableUnusedPins ()
 * @brief disable all unused pins 
*/
void disableUnusedPins ()
{
    for ( int i = 0; i < N_UNUSED_PINS; i ++ ) {
        pinMode ( PINS_UNUSED [i], 0 );
    }
}

/**
 * @fn setupSensors ()
 * @brief Initialise global variables, Start polling timers and attach ISRs
 */
void setupSensors () 
{
    // configure DIG INPUTS
    for ( int i = 0; i < N_DIG_IN; i ++ ) {
        if ( FLAG_DIG_IN_POPULATED [i] )
            pinMode ( PINS_DIG_IN [i], INPUT );
        else
            pinMode ( PINS_DIG_IN [i], INPUT_DISABLE );
    }

    // configure ANG INPUTS
    for ( int i = 0; i < N_ANG_IN; i ++ ) {
        if ( FLAG_ANG_IN_POPULATED [i] )
            pinMode ( PINS_ANG_IN [i], INPUT );
        else
            pinMode ( PINS_ANG_IN [i], INPUT_DISABLE );
    }

    // initialise buffers
    _dig_readings = new uint8_t  [N_DIG_IN] { 0 };
    _ang_readings = new uint16_t [N_ANG_IN] { 0 };

}// setupSensors ()


/** 
 * @fn setupTempSensors ()
 * @brief Initialize variables for polling. Start polling interrupt timer and attach ISR
 */
void setupTempSensors ()
{
    // Allocate memory for buffers on heap
    _temps         = new float [N_THERMISTOR] { 0 };
    _temp_readings = new float [N_THERMISTOR] { 0 };

    // Initialize variables
    _flag_poll_thermistor = false;
    _n_accrued_temp_readings = 0;
    
    // set pinMode for temp sensor pins and populate buffers
    for ( int i = 0; i < N_THERMISTOR; i ++ ) { 
        
        if ( FLAG_THERMISTOR_POPULATED [i] )
            pinMode ( PINS_THERMISTOR [i], INPUT );         // treat populated inputs as normal
        else
            pinMode ( PINS_THERMISTOR [i], INPUT_DISABLE ); // disable un-used inputs
        
        // initialise buffers
        _temps [i] = 0;
        _temp_readings [i] = 0;
    }

}// setupTempSensors ()

/**
 * @fn setupPWM
 * @brief set pinMode for all pwm pins and start pwm timers
*/
void setupPWM ()
{
    // initialise variables on heap
    _pwr_pwm_freq = PWR_PWM_FREQ;
    _sig_pwm_freq = SIG_PWM_FREQ;
    _sig_pwm_duty = new float [N_SIG_PWM] { 0 };
    _pwr_pwm_duty = new float [N_PWR_PWM] { 0 };
    _pwr_pwm_enabled = new bool [N_PWR_PWM] { false };

    for ( uint8_t i = 0; i < N_SIG_PWM; i ++ )
    {
        // instantiate hardware library instance
        _hwpwm_sig [i] = new Teensy_PWM ( PINS_SIG_PWM [i], _sig_pwm_freq, 0.0f );
    }
    for ( uint8_t i = 0; i < N_PWR_PWM; i ++ )
    {
        // setup enable pins
        pinMode ( PINS_PWR_PWM_EN [i], OUTPUT );

        // instantiate hardware library instance
        _hwpwm_pwr [i] = new Teensy_PWM ( PINS_PWR_PWM_S [i], _pwr_pwm_freq, 0.0f );
    }

    // disable both PWM circuits
    disablePWRPWM ();
    disableSIGPWM ();

}// setupPWM ()


/**
 * @fn setupCANTransceivers ()
 * @brief Initialise FlexCAN_T4 objects using defined CAN configuration
*/
void setupCAN ()
{
    if ( FLAG_CAN_POPULATED [0] )
        canInit ( _can1, CAN1_BAUD, can1Receive );
    if ( FLAG_CAN_POPULATED [1] )
        canInit ( _can2, CAN2_BAUD, can2Receive );
    if ( FLAG_CAN_POPULATED [2] )
        canInit ( _can3, CAN3_BAUD, can3Receive );
}// setupCAN ()


/**
 * @fn setupPollingTimers ()
 * @brief Starts the IntervalTimer objects and attaches ISR. Timer interval is calculated using the defined polling frequencies.
*/
void setupPollingTimers ()
{
    // start timer for temp sensor polling
    uint32_t temp_sensor_polling_delay_us = 1000000 / THERMISTOR_POLL_FREQ;
    _thermistor_poll_timer.begin ( pollTempSensorsISR, temp_sensor_polling_delay_us );

    // start timer for sensor polling with an interval to match the polling freq
    uint32_t sensor_polling_delay_us = 1000000 / SNSR_POLLING_FREQ;
    _snsr_poll_timer.begin ( pollSensorsISR, sensor_polling_delay_us ); 

}// startPollingISRs ()

/**
 * @fn initTempRanges ()
 * @brief instantiates structs representing temperature ranges using configured variables
*/
void initTempRanges ()
{
    _temp_range_batt = new TempRange ( 
        SAFETY_TEMP_BATT_MIN_OK,
        SAFETY_TEMP_BATT_MAX_OK,
        SAFETY_TEMP_BATT_MIN_CRIT,
        SAFETY_TEMP_BATT_MAX_CRIT
    );

    _temp_range_chgr = new TempRange ( 
        SAFETY_TEMP_CHGR_MIN_OK,
        SAFETY_TEMP_CHGR_MAX_OK,
        SAFETY_TEMP_CHGR_MIN_CRIT,
        SAFETY_TEMP_CHGR_MAX_CRIT
    );

    _temp_range_batt = new TempRange ( 
        SAFETY_TEMP_RES_MIN_OK,
        SAFETY_TEMP_RES_MAX_OK,
        SAFETY_TEMP_RES_MIN_CRIT,
        SAFETY_TEMP_RES_MAX_CRIT
    );
}// initTempRanges ()

//---------------------------   [   POLLING ISRs   ]  ----------------------------------//


// Interval timers only need to set a flag
void pollTempSensorsISR () { _flag_poll_thermistor = true; }
void pollSensorsISR ()     { _flag_poll_snsr       = true; }


//---------------------------   [   POLL FN HANDLERS   ]  ----------------------------------//


/**
 * @fn handlePollingMethods
 * @brief Manages volatile polling flags accessed by polling timer ISRs and calls all sensor polling methods accordingly.
*/
void handlePollingMethods ()
{
    // duplicate variables for access to volatile shared variables
    bool flag_poll_thermistor_cpy = false;
    bool flag_poll_snsr_cpy       = false;
    
    // disable interrupts whilst using volatile memory
    noInterrupts (); 

    // reset flags
    // ISRs only set the flag true, if the flag was read as true, it needs to be reset
    if ( _flag_poll_thermistor ) { 
        flag_poll_thermistor_cpy = true; 
        _flag_poll_thermistor = false;
    }

    if ( _flag_poll_snsr ) { 
        flag_poll_snsr_cpy = true; 
        _flag_poll_snsr = false;
    }
    
    // re-enable interrupts after handling volatile memory
    interrupts (); 

    // call polling methods according to flags
    if ( flag_poll_thermistor_cpy ) { handleTempSensorPolling (); }
    if ( flag_poll_snsr_cpy )       { handleSensorPolling (); }

}// handlePollingMethods ()


/**
 * @fn handleTempSensorPolling ()
 * @brief Manages reading the analogue 
*/
void handleTempSensorPolling ()
{
    _n_accrued_temp_readings ++; 
    pollTempSensors ();


    // check if enough readings are stored so that they can be averaged and converted to temperatures
    if ( _n_accrued_temp_readings + 1 == THERMISTOR_READING_BUFFER_SIZE ) {
        
        // reset sample counter
        _n_accrued_temp_readings = 0;
        
        // calculate temperatures using averaged readings and clear buffer
        for ( int i = 0; i < N_THERMISTOR; i ++ ) { 

            // average accumulated readings in buffer and reset 
            float avg = _temp_readings [i] / (float)THERMISTOR_READING_BUFFER_SIZE;
            _temp_readings [i] = 0;     
            
            // calculate the temperature using averaged sensor reading
            _temps [i] = orionNTC10K_readingToTemp ( avg, THERMISTOR_VOLTAGE_DIV_RESISTANCE ) ; 
        }
    }
}// handleTempSensorPolling ()


/**
 * @fn handleSensorPolling ()
 * @brief Function calls all poll functions for the various configured inputs
*/
void handleSensorPolling () 
{ 
    pollDigitalInputs ();
    pollAnalogueInputs ();
}


//---------------------------   [   POLL FNs   ]  ----------------------------------//


/**
 * @fn pollTempSensors ()
 * @brief Accumulates ADC readings from all populated temp sensors
 * @warning No reading will be taken unless the relevant FLAG_THERMISTOR_POPULATED is configured
*/
void pollTempSensors () 
{
    for ( uint8_t i = 0; i < N_THERMISTOR; i ++ ) { 
        if ( FLAG_THERMISTOR_POPULATED [i] ) {
            // take ADC reading and accumulate in buffer
            _temp_readings [i] += analogRead ( PINS_THERMISTOR [i] ); 
        }
    }
}// pollTempSensors ()

/**
 * @fn pollDigitalInputs ()
 * @brief Take a reading from all populated digital inputs and store results in buffer
*/
void pollDigitalInputs ()
{
    for ( uint8_t i = 0; i < N_DIG_IN; i ++ )
    {
        if ( FLAG_DIG_IN_POPULATED [i] ) {
            _dig_readings [i] = digitalRead ( PINS_DIG_IN [i] );
        }
    }
}// pollDigitalInputs ()

/**
 * @fn pollAnalogueInputs ()
 * @brief Take readings from all populated analogue inputs and store results in buffer
*/
void pollAnalogueInputs ()
{
    for ( uint8_t i = 0; i < N_ANG_IN; i ++ ) {
        if ( FLAG_ANG_IN_POPULATED [i] ) {
            _ang_readings [i] = analogRead ( PINS_ANG_IN [i] );
        }
    }
}// pollAnalogueInputs ()


//---------------------------   [   PWM   ]  ----------------------------------//

/**
 * @fn enablePWRPWM ()
 * @brief Enables all PWR PWM circuits. 
 *        Disable the transistor used to pull enable signal to GND. Enables PWR PWM output
*/
void enablePWRPWM ()
{
    for ( uint8_t i = 0; i < N_PWR_PWM; i ++ )
    {
        digitalWrite ( PINS_PWR_PWM_EN [i], LOW );
        _pwr_pwm_enabled [i] = true;
    }
}

/**
 * @fn enablePWRPWM ()
 * @brief Enables specified 0 indexed PWM circuit. 
 *        Disable the transistor used to pull enable signal to GND. Enables PWR PWM output
*/
void enablePWRPWM ( const uint8_t& idx )
{
    digitalWrite ( PINS_PWR_PWM_EN [idx], LOW );
    _pwr_pwm_enabled [idx] = true;
}

/**
 * @fn enablePWRPWM ()
 * @brief Disables all PWR PWM circuits. 
 *        Enables the transistor used to pull enable signal to GND. Enables PWR PWM output
*/
void disablePWRPWM ()
{
    for ( uint8_t i = 0; i < N_PWR_PWM; i ++ )
    {
        digitalWrite ( PINS_PWR_PWM_EN [i], HIGH );
        _pwr_pwm_enabled [i] = false;
    }
}

/**
 * @fn enablePWRPWM ()
 * @brief Disables specified PWR PWM circuits. 
 *        Eanble the transistor used to pull enable signal to GND. Enables PWR PWM output
*/
void disablePWRPWM ( const uint8_t& idx )
{
    digitalWrite ( PINS_PWR_PWM_EN [idx], HIGH );
    _pwr_pwm_enabled [idx] = false;
}


/**
 * @fn disableSIGPWM ()
 * @brief disables the outputs of SIG PWM circuits by setting the driver signal constant high > pulls output to GND
*/
void disableSIGPWM ()
{
    for ( uint8_t i = 0; i < N_SIG_PWM; i ++ )
    {
        digitalWrite ( PINS_SIG_PWM [i], HIGH );
    }
}

/**
 * @fn setSIGPWM
 * @brief Wrapper for Teensy_PWM class calls
 * @warning Setting the frequency will change both SIG PWM  freq
 * 
 * @arg {uint8_t}   idx  - index of pwm line to drive 
 * @arg {float}     freq - frequency to operate PWM signals
 * @arg {float}     duty - duty cycle to set 
*/
void setSIGPWM ( const uint8_t& idx, const float& freq, const float& duty )
{
    // invert duty cycle if needed
    float _duty = 0.0f;
    if ( INVERT_SIG_PWM )
        _duty = 100 - duty;
    else
        _duty = duty;

    _hwpwm_sig [idx]->setPWM ( PINS_SIG_PWM [idx], _sig_pwm_freq, _duty );
}
    

/**
 * @fn setPWRPWM ()
 * @warning changing the frequency for a power pwm pin will change the frequency of both pins
 * @arg {uint8_t}   idx  - pwm index ( 0 to N_PWR_PWM - 1 )
 * @arg {float}     freq - frequency to operate PWM signals
 * @arg {float}     duty - duty cycle to set 
*/
void setPWRPWM ( const uint8_t& idx, const float& freq, const float& duty )
{
    // enable PWR PWM 
    if ( !_pwr_pwm_enabled [idx] )
    {
        enablePWRPWM ( idx );
    }

    //
    _hwpwm_pwr [idx]->setPWM ( PINS_PWR_PWM_S [idx], freq, duty );
}


//---------------------------   [   Serial   ]  ----------------------------------//


/**
 * @fn serialDisplayInit ()
 * @brief Display general info and a welcome message over the serial
*/
void serialDisplayInit ()
{
    Serial.println ( "*****************************************************" );
    
    sprintf ( _serial_msg_bfr, "Unique EVS Climate Control for TEENSY 4.1\n" );
    
    // add software version
    sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "SOFTWARE VERSION: %f\n", CC_SOFTWARE_VERSION );
    Serial.println ( _serial_msg_bfr );

    Serial.println ( "*****************************************************" );
    
    // temp sensors configuration
    Serial.println ( "TEMP SENSORS CONFIGURATION:" );
    serialDisplayConfigStatus ( N_THERMISTOR, FLAG_THERMISTOR_POPULATED );

    // digital input configuration
    Serial.println ( "DIGITAL INPUT CONFIGURATION:" );
    serialDisplayConfigStatus ( N_DIG_IN, FLAG_THERMISTOR_POPULATED );

    // analogue input configuration
    Serial.println ( "ANALOGUE INPUT CONFIGURATION:" );
    serialDisplayConfigStatus ( N_ANG_IN, FLAG_ANG_IN_POPULATED );

}// serialDisplayInit ()

void serialDisplayConfigStatus ( int len, bool* flags )
{
    sprintf ( _serial_msg_bfr, "*****************************************************\n" );
    for ( uint8_t i = 0; i < len; i ++ ) {
        
        if ( flags [i] )
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "[%d]\tY\n", i + 1 );
        else
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "[%d]\tN\n",  i + 1 );
    }
    sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "*****************************************************" );

    Serial.println ( _serial_msg_bfr );
}

/**
 * @fn serialDisplay
 * @brief Display to the serial console the current board state.
*/
void serialDisplay ()
{

#if DISPLAY_DIG_INPUTS

    sprintf ( _serial_msg_bfr, "Digital Inputs:\n[" );
    for ( uint8_t i = 0; i < N_DIG_IN; i ++ ) { 
        if ( FLAG_DIG_IN_POPULATED [i] )
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t%d", _dig_readings [i] ); 
        else
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t!N.C!" );
    }

    sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t]\n" );
    Serial.print ( _serial_msg_bfr );

#endif

#if DISPLAY_ANG_INPUTS
    // Temp Readings:
    // [ R0, R1, R2, ..., Rn ]
    sprintf ( _serial_msg_bfr, "Analogue Inputs:\n[" );
    for ( uint8_t i = 0; i < N_ANG_IN; i ++ ) { 
        if ( FLAG_ANG_IN_POPULATED [i] )
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t%f", ( (float)_ang_readings [i] / (float)ANG_IN_MAX_RES ) * 13.0f ); 
        else
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t!N.C!" );
    }

    sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t]\n" );
    Serial.print ( _serial_msg_bfr );
#endif

#if DISPLAY_TEMPERATURES

    // Temp Readings:
    // [ R0, R1, R2, ..., Rn ]
    sprintf ( _serial_msg_bfr, "Temperature Readings:\n[" );
    for ( uint8_t i = 0; i < N_THERMISTOR; i ++ ) { 
        if ( FLAG_THERMISTOR_POPULATED [i] )
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t%.2f", _temps [i] ); 
        else
            sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t!N.C!" );
    }
    sprintf ( _serial_msg_bfr + strlen ( _serial_msg_bfr ), "\t]\n" );
    Serial.print ( _serial_msg_bfr );

#endif


}// serialDisplay ()

