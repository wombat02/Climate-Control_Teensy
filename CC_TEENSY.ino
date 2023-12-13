/*
Version 0.0.1

Teensy 4.1 Climate Control System
Author: Brodie Collinson Davison (wombat02)

Copyright (c) 2023 Unique EVS

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

// Interrupts
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pins.h"
#include "config.h"
#include "orion_temp_sensor.h"

// SERIAL DISPLAY
uint16_t serial_display_delay_ms;         // time between sending serial messages
uint32_t next_serial_display_time_ms;     // time to send the next serial message

// TEMP SENSORS
float* temps;                             // actual temperature measurements
float* temp_readings;                     // buffer to calculate averages of temp sensor readings  
uint8_t n_accrued_temp_readings;          // number of readings placed into the readings buffer

volatile bool flag_poll_thermistor;        // indicates whether the temperature sensors are due to be polled
volatile bool flag_poll_snsr;
IntervalTimer thermistor_poll_timer;
IntervalTimer snsr_poll_timer;

uint8_t target_temp;

//----------------------------      FUNCTION DECLARATIONS   ----------------------------//
void setupSensors       ();
void setupTempSensors   ();

void pollTempSensorsISR ();
void pollTempSensors    ();
void handleTempSensorPolling  ();

void pollSensorsISR      ();
void pollTargetTemp      ();
void handleSensorPolling ();

void serialDisplay ();

//----------------------------      PROGRAM SETUP   ----------------------------//

void setup() {

    setupTempSensors ();
    setupSensors     ();

    // set onboard LED ON to indicate uC is running
    if ( ENABLE_ONBOARD_LED ) { pinMode ( 13, OUTPUT ); digitalWrite ( 13, HIGH ); }

    // enable serial comms and initialise msg buffer / timing variables
    if ( ENABLE_SERIAL_OUTPUT ) {
        serial_display_delay_ms = 1000 / SERIAL_DISPLAY_FREQ;
        next_serial_display_time_ms = 0;
        Serial.begin ( SERIAL_BAUD );
        
        delay ( 100 );
        Serial.println ( "Serial enabled" );
    }

}// setup ()

//----------------------------      PROGRAM LOOP   ----------------------------//

void loop() {

    // get the current millisecond
    uint32_t cur_millis = millis ();

    // duplicate variables for access to volatile shared variables
    bool poll_temp_snsrs = false;
    bool poll_snsrs = false;
    
    noInterrupts (); // disable interrupts whilst using volatile memory
    // copy variables into non-volatile storage
    poll_temp_snsrs = flag_poll_thermistor;
    poll_snsrs = flag_poll_snsr;

    // reset flags 
    if ( poll_temp_snsrs ) { flag_poll_thermistor = false; }
    if ( poll_snsrs ) { flag_poll_snsr = false; }
    interrupts (); 

    // manage polling of temp sensors
    if ( poll_temp_snsrs ) { handleTempSensorPolling (); }
    // manage polling of other inputs
    if ( poll_snsrs ) { handleSensorPolling (); }

    // SERIAL DISPLAY
    if ( ENABLE_SERIAL_OUTPUT ) {
        if ( cur_millis >= next_serial_display_time_ms ) {
            serialDisplay ();
            next_serial_display_time_ms = cur_millis + serial_display_delay_ms;
        }
    }
}// loop ()

//----------------------------      FUNCTIONS   ----------------------------//

///     TEMP SENSORS        ///
void pollTempSensorsISR () { flag_poll_thermistor = true; }
void pollTempSensors () 
{
    n_accrued_temp_readings ++; // increment poll count and read values from all sensor pins
    for ( int i = 0; i < N_THERMISTOR; i ++ ) { temp_readings [i] += analogRead ( THERMISTOR_PINS [i] ); }
}

/* setupTempSensors ()
 * Allocate temp / reading buffers on heap
 * Initialize variables for polling
 * Start polling interrupt timer and attach ISR
 */
void setupTempSensors ()
{
    // Allocate memory for buffers on heap
    temps         = new float [N_THERMISTOR];
    temp_readings = new float [N_THERMISTOR];

    // Initialize variables
    flag_poll_thermistor = false;
    n_accrued_temp_readings = 0;
    
    // set pinMode for temp sensor pins and populate buffers
    for ( int i = 0; i < N_THERMISTOR; i ++ ) { 
        
        if ( THERMISTOR_INPUT_POPULATED_FLAGS [i] )
            pinMode ( THERMISTOR_PINS [i], INPUT_PULLUP );
        else
            pinMode ( THERMISTOR_PINS [i], INPUT ); 
        
        temps [i] = 0;
        temp_readings [i] = 0;
    }

    // start timer for temp sensor polling
    int temp_sensor_polling_delay_us = 1000000 / THERMISTOR_POLL_FREQ;
    thermistor_poll_timer.begin ( pollTempSensorsISR, temp_sensor_polling_delay_us ); 
}

/* handleTempSensorPolling ()
 *  Handles polling and averaging of polled values for all temp sensors defined in pins.h
 */
void handleTempSensorPolling ()
{
    pollTempSensors ();

    // check if reached the number of samples required to average readings 
    if ( n_accrued_temp_readings + 1 == N_THERMISTOR_POLLS ) {
        
        // reset sample counter
        n_accrued_temp_readings = 0;
        
        // calculate temperatures using averaged readings and clear buffer
        for ( int i = 0; i < N_THERMISTOR; i ++ ) { 
            temps [i] = orionNTC10K_readingToTemp ( temp_readings [i] / (float)N_THERMISTOR_POLLS ) ; 
            temp_readings [i] = 0;     
        }
    }
}

///     SENSOR POLLING      ///

void pollSensorsISR () { flag_poll_snsr = true; }

/* setupSensors ()
 * Initialise global variables
 * Start polling timer and attach ISR
 */
void setupSensors ()
{
    // setup pins
    pinMode ( TEMPCTL_POT_PIN, INPUT );
    
    // initialise variables
    target_temp = 0;

    // start timer for sensor polling with an interval to match the polling freq
    int sensor_polling_delay_us = 1000000 / SNSR_POLLING_FREQ;
    snsr_poll_timer.begin ( pollSensorsISR, sensor_polling_delay_us ); 
}

/* pollTargetTemp ()
 * reads the value of the temp setting potentiometer and converts it to a range [ MIN_TARGET_TEMP, MAX_TARGET_TEMP ]
 */
void pollTargetTemp ()
{
    target_temp = map ( analogRead ( TEMPCTL_POT_PIN ), 0, 1023, MIN_TARGET_TEMP, MAX_TARGET_TEMP ); 
}

/* handleSensorPolling ()
 * Poll all sensors / inputs
 */
void handleSensorPolling () 
{ 
    pollTargetTemp (); 
}


///         MISC            ///
/*
    serialDisplay ()
    Print the current state of system
*/
void serialDisplay ()
{
    // create buffer for serial messages
    char msgbuf [512];
    
    // TARGET TEMP
    sprintf ( msgbuf, "Temp setting: %d\n", target_temp );
    
    // TEMP SENSORS
    sprintf ( msgbuf + strlen(msgbuf), "Temp Readings:\n[" );
    for ( int i = 0; i < N_THERMISTOR; i ++ ) { sprintf ( msgbuf + strlen(msgbuf), "\t%.2f", temps [i] ); }
    sprintf ( msgbuf + strlen(msgbuf), "\t]\n" );
    Serial.print ( msgbuf );
}