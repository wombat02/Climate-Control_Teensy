#ifndef ORION_TEMP_SENSOR_H
#define ORION_TEMP_SENSOR_H

// Values for small black temp sensors provided with ORION BMS kits
#define ORION_NTC10K_R0 10000    // 10K ohm @ room temp
#define ORION_NTC10K_T0 293.15   // *K (25 *C)
#define ORION_NTC10K_BETA 3380   // Beta value for 25/50 R-T curve

/**
 * @fn orionNTC10K_readingToTemp ()
 * @brief Convert the measured resistance of an ORION NTC10K thermistor 
 * using adc reading from node A to a temperature in *C
 *  
 *  (VCC) ----[DIVIDER]---(*)---[NTC10k]--- (GND)
 *                         |
 *                       (ADC)
 * 
 * @arg {float} reading - ADC reading to be converted to temperature
*/
float orionNTC10K_readingToTemp ( const float& reading, const uint32_t& dividerResistance )
{
  float temp = reading;

  // Calculate resistance of thermistor using voltage divider
  // Resistance(Temp) = Resistance(div) / ( (1023 / adc) - 1 )
  temp = ( 1023.0 / temp ) - 1;             // (1023 / adc) - 1
  temp =  (float)dividerResistance / temp;  // 20000 / ( 1023/adc - 1 ) 

  // Stein Heart Eqn ( condensed form only using beta )
  // 1/T = 1/To + 1/B * ln ( R / Ro )
  temp  = temp / (float)ORION_NTC10K_R0;    // R / Ro
  temp  = log ( temp );                     // ln ( R / Ro )
  temp /= (float)ORION_NTC10K_BETA;         // (1/b) * ln ( R / Ro )
  temp += 1.0 / (float)ORION_NTC10K_T0;     // add 1 / To
  temp  = 1.0 / temp;                       // invert 
  temp -= 273.15;                           // *K --> *C

  return temp;
}// orionNTC10K_readingToTemp ()

#endif 