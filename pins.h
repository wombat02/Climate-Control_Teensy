#ifndef PINS_H
#define PINS_H

//------------------------      INPUTS     --------------------------//

//  DIGITAL INPUTS          
#define N_DIG_IN 8
uint8_t DIG_IN_PINS [N_DIG_IN] = { 2, 3, 4, 5, 6, 7, 8, 9 };

//  ANALOGUE INPUTS         
#define N_ANG_IN 4
uint8_t ANG_IN_PINS [N_ANG_IN] = { A16, A17, A14, A15 };

//  NTC 10K THERMISTOR INPUTS           
#define N_THERMISTOR 6
uint8_t THERMISTOR_PINS [N_THERMISTOR] = { A0, A1, A2, A3, A4, A5 };

//------------------------      OUTPUTS     --------------------------//

//  DIGITAL OUTPUTS             
#define N_DIG_OUT 4
uint8_t DIG_OUT_PINS [N_DIG_OUT] = { 24, 12, 11, 10 };

//  RELAY OUTPUTS               
#define N_RELAY 5
uint8_t RELAY_PINS [N_RELAY] = { 26, 27, 35, 34, 33 };

//  SIGNAL PWM OUTPUTS          
#define N_SIG_PWM 2
uint8_t SIG_PWM_PINS [N_SIG_PWM] = { 28, 29 };

//  CANBUS INTERFACE PINS          
#define CAN1_RX_PIN 23
#define CAN1_TX_PIN 22

#define CAN2_RX_PIN 30
#define CAN2_TX_PIN 31

#define CAN3_RX_PIN 0
#define CAN3_TX_PIN 1

#endif 