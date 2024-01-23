
#ifndef CANBUS_H
#define CANBUS_H

#include <FlexCAN_T4.h>

//---------------------------           CAN CONFIGURATION         -------------------------------//

#define DEFAULT_CAN_BAUD 500000

#define CAN1_BAUD DEFAULT_CAN_BAUD
#define CAN2_BAUD DEFAULT_CAN_BAUD
#define CAN3_BAUD DEFAULT_CAN_BAUD

// externally defined in CC_TEENSY.ino
extern FlexCAN_T4 <CAN1, RX_SIZE_256, TX_SIZE_16> can1;
extern FlexCAN_T4 <CAN2, RX_SIZE_256, TX_SIZE_16> can2;
extern FlexCAN_T4 <CAN3, RX_SIZE_256, TX_SIZE_16> can3;

/**
 * @fn canInit<CANType>
 * 
 * Initialise CAN transciever using FlexCAN_T4
 * @arg {CANType&} can - instance of FlexCAN_T4 object
 * @arg {int} baudrate - baud to run CAN at (Kb/s)
 * @arg {*fn(const CAN_message_t&)} onReceiveFn - pointer to function that is called to handle received messages
*/
template <typename CANType>
void canInit(CANType& can, int baudrate, void (*onReceiveFn)(const CAN_message_t&)) {
  
  // initialise can line at specified baud
  can.begin ();
  can.setBaudRate ( baudrate );
  // enable FIFO queuing & interrupts
  can.enableFIFO ();
  can.enableFIFOInterrupt ();
  // assign receive function interrupt callback method
  can.onReceive ( onReceiveFn );
}

//---------------------------           CAN MSG RECEIVE FUNCTIONS         -------------------------------//

/**
 * @fn can1Receive
 * @brief function is called every time a message is received on the CAN interface
*/
void can1Receive ( const CAN_message_t& msg )
{
    // add functionality for CAN1 here
}// can1Receive ()

/**
 * @fn can2Receive
 * @brief function is called every time a message is received on the CAN interface
*/
void can2Receive ( const CAN_message_t& msg  )
{
    // add functionality for CAN2 here
}// can2Receive ()

/**
 * @fn can3Receive
 * @brief function is called every time a message is received on the CAN interface
*/
void can3Receive ( const CAN_message_t& msg  )
{
    // add functionality for CAN3 here
}// can3Receive ()


#endif