#ifndef FSM_H
#define FSM_H

#include <etl/fsm.h>
#include "pins.h"

extern const etl::message_router_id_t FSM_ROUTER = 0;
extern char* _serial_msg_bfr;

//--------------------------------- [   FSM EVENTS   ]   ---------------------------------//

struct EventID {
    enum {
        UPDATE,
        BTN_AC,
        BTN_AUTO,
        BTN_HEATER,
        CHARGE_ON,
        CHARGE_OFF,
        TEMP_FAULT
    };
};

class eUPDATE       : public etl::message<EventID::UPDATE>     {};
class eBTN_AC       : public etl::message<EventID::BTN_AC>     {};
class eBTN_AUTO     : public etl::message<EventID::BTN_AUTO>   {};
class eBTN_HEATER   : public etl::message<EventID::BTN_HEATER> {};
class eCHG_ON       : public etl::message<EventID::CHARGE_ON>  {};
class eCHG_OFF      : public etl::message<EventID::CHARGE_OFF> {};
class eTEMP_FAULT   : public etl::message<EventID::TEMP_FAULT> {
public:
    const char* msg;
    TempRange tempRange;

    eTEMP_FAULT ( const char* _msg, const TempRange& _tempRange )
    : msg (_msg ),
    tempRange ( _tempRange )  
    {}
};

//--------------------------------- [   FSM STATES   ]   ---------------------------------//


struct StateID {

    enum {
        IDLE,
        ON,
        AUTO,
        CHARGING,
        TEMP_FAULT,
        N_STATES
    };
};

const char* stateIDStrings [ StateID::N_STATES] = {
    "IDLE",
    "ON",
    "AUTO",
    "CHARGING",
    "TEMP-FAULT"
}; 


//--------------------------------- [   FSM CLASS   ]   ---------------------------------//

class CCFSM : public etl::fsm
{
public:

    // provide router using default constructor
    CCFSM ()
    : fsm ( FSM_ROUTER )
    { }
    
    void LogUnkownEvent ( const etl::imessage& msg )
    {
        // get states
        etl::fsm_state_id_t cur_state = get_state ().get_state_id ();
        etl::fsm_state_id_t req_state = static_cast<etl::fsm_state_id_t> ( msg.get_message_id () );

        sprintf ( _serial_msg_bfr, "[ FSM ERR ] Unknown Event:\nCUR:%s\nREQ:%s", stateIDStrings[cur_state], stateIDStrings [req_state] );
        Serial.println ( _serial_msg_bfr );
    }
};


// -------------------------------------     [ sIdle ]     --------------------------------------------------- //

class sIdle : public etl::fsm_state <CCFSM, sIdle, StateID::IDLE, 
                                    eUPDATE, 
                                    eBTN_AC, 
                                    eBTN_AUTO, 
                                    eTEMP_FAULT, 
                                    eCHG_ON> 
{
public:

    // **************    on_enter/exit   ********************* 
    etl::fsm_state_id_t on_enter_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] ENTER", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );

        return No_State_Change;
    }

    void on_exit_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] EXIT", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );
    }
    // **************    on_enter/exit   ********************* 

    /**
     * @fn on_event_unknown
    */
    etl::fsm_state_id_t on_event_unknown ( const etl::imessage& msg )
    {
        get_fsm_context ().LogUnkownEvent ( msg );
        return STATE_ID;
    }
    
    
    /** 
     * @arg eUPDATE
    */
    etl::fsm_state_id_t on_event ( const eUPDATE& e )
    {
        Serial.println ( "UPDATE CALLED FROM IDLE" );
        return No_State_Change;
    }// eUPDATE 

    /**
     * @arg eBTN_AC
    */
    etl::fsm_state_id_t on_event ( const eBTN_AC& e )
    {
        return StateID::ON;
    }// eBTN_AC

    /**
     * @arg eBTN_AUTO
    */
    etl::fsm_state_id_t on_event ( const eBTN_AUTO& e )
    {
        return StateID::AUTO;
    }// eBTN_AUTO

    /**
     * @arg eTEMP_FAULT
    */
    etl::fsm_state_id_t on_event ( const eTEMP_FAULT& e )
    {
        return StateID::TEMP_FAULT;
    }// eTEMP_FAULT

    /**
     * @arg eCHG_ON
    */
    etl::fsm_state_id_t on_event ( const eCHG_ON& e )
    {
        return StateID::CHARGING;
    }// eCHG_ON


};

// -------------------------------------     [ sON ]     --------------------------------------------------- //

class sON : public etl::fsm_state <CCFSM, sON, StateID::ON, 
                                  eBTN_AC, 
                                  eBTN_AUTO, 
                                  eBTN_HEATER, 
                                  eCHG_ON, 
                                  eTEMP_FAULT>
{
public:

    etl::fsm_state_id_t on_enter_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] ENTER", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );

        return No_State_Change;
    }

    void on_exit_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] EXIT", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );
    }

    etl::fsm_state_id_t on_event ( const eBTN_AC& e )
    {
        // ON -> IDLE
        return StateID::IDLE;
    }

    etl::fsm_state_id_t on_event ( const eBTN_AUTO& e )
    {
        // ON -> AUTO
        return StateID::AUTO;
    }

    etl::fsm_state_id_t on_event ( const eBTN_HEATER& e )
    {
        // TURN ON HEATER
        return STATE_ID;
    }

    etl::fsm_state_id_t on_event ( const eCHG_ON& e )
    {
        // ON -> CHG
        return StateID::CHARGING;
    }

    etl::fsm_state_id_t on_event ( const eTEMP_FAULT& e )
    {
        // ON -> TEMP FLT
        return StateID::TEMP_FAULT;
    }
    
    etl::fsm_state_id_t on_event_unknown ( const etl::imessage& msg )
    {
        get_fsm_context ().LogUnkownEvent ( msg );
        return STATE_ID;
    }
};

// -------------------------------------     [ sAuto ]     --------------------------------------------------- //

class sAuto : public etl::fsm_state <CCFSM, sAuto, StateID::AUTO, 
                                    eBTN_AUTO, 
                                    eBTN_AC, 
                                    eCHG_ON, 
                                    eTEMP_FAULT>
{
public:

    etl::fsm_state_id_t on_enter_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] ENTER", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );

        return No_State_Change;
    }

    void on_exit_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] EXIT", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );
    }

    etl::fsm_state_id_t on_event ( const eBTN_AUTO& e )
    {
        // AUTO -> ON
        return StateID::ON;
    }

    etl::fsm_state_id_t on_event ( const eBTN_AC& e )
    {
        // AUTO -> IDLE
        return StateID::IDLE;
    }

    etl::fsm_state_id_t on_event ( const eCHG_ON& e )
    {
        // AUTO -> CHG
        return StateID::CHARGING;
    }

    etl::fsm_state_id_t on_event ( const eTEMP_FAULT& e )
    {
        // AUTO -> TEMP FLT
        return StateID::TEMP_FAULT;
    }
    
    etl::fsm_state_id_t on_event_unknown ( const etl::imessage& msg )
    {
        get_fsm_context ().LogUnkownEvent ( msg );
        return STATE_ID;
    }
};

// -------------------------------------     [ sCharging ]     --------------------------------------------------- //

class sCharging : public etl::fsm_state <CCFSM, sCharging, StateID::CHARGING, 
                                        eCHG_OFF, 
                                        eTEMP_FAULT>
{
public:

    etl::fsm_state_id_t on_enter_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] ENTER", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );

        return No_State_Change;
    }

    void on_exit_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] EXIT", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );
    }
    
    etl::fsm_state_id_t on_event ( const eCHG_OFF& e )
    {
        // done charging -> IDLE
        return StateID::IDLE;
    }

    etl::fsm_state_id_t on_event ( const eTEMP_FAULT& e )
    {
        return StateID::TEMP_FAULT;
    }

    etl::fsm_state_id_t on_event_unknown ( const etl::imessage& msg )
    {
        get_fsm_context ().LogUnkownEvent ( msg );
        return STATE_ID;
    }
};

// -------------------------------------     [ sTempFault ]     --------------------------------------------------- //

class sTempFault : public etl::fsm_state <CCFSM, sTempFault, StateID::TEMP_FAULT,
                                         eUPDATE>
{
public:

    etl::fsm_state_id_t on_enter_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] ENTER", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );

        return No_State_Change;
    }

    void on_exit_state ()
    {
        sprintf ( _serial_msg_bfr, "[%s] EXIT", stateIDStrings [STATE_ID] );
        Serial.println ( _serial_msg_bfr );
    }
    
    /// @todo internal logic to get out of temp fault state
    
    etl::fsm_state_id_t on_event ( const eUPDATE& e )
    {
        return No_State_Change;
    }
    
    etl::fsm_state_id_t on_event_unknown ( const etl::imessage& msg )
    {
        get_fsm_context ().LogUnkownEvent ( msg );
        return STATE_ID;
    }
};



#endif