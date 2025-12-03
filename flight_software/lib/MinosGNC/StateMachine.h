//#include "util.h"
#pragma once
#include "Arduino.h"
#include "minos_util.h"

// 500m in mm
// Very normal units (TM)
#define MAIN_DEPLOY_ALT 500e3

enum flight_fsm_e : uint32_t {  
    ABORT,
    IDLE,
    LAUNCH, 
    COAST, 
    DROGUE, 
    MAINS,

};

 typedef struct {
    flight_fsm_e state;
    uint32_t init_time;
} flight_fsm_t;

class FlightStateMachine {
    public:
        bool init(flight_fsm_e init_state);
        bool update(SensorMap& sensors, ActuatorMap& actuators);
        bool net_command(flight_fsm_e command);
        flight_fsm_e get_state() { return current_state.state; }
        
    private:
        flight_fsm_t current_state;

        flight_fsm_e net_cmd_state;

        bool change_state(flight_fsm_e new_state);

        bool case_idle(SensorMap& sensors, ActuatorMap& actuators);
        bool case_launch(SensorMap& sensors, ActuatorMap& actuators);
        bool case_coast(SensorMap& sensors, ActuatorMap& actuators);
        bool case_drogue(SensorMap& sensors, ActuatorMap& actuators);
        bool case_mains(SensorMap& sensors, ActuatorMap& actuators);
        bool case_abort(SensorMap& sensors, ActuatorMap& actuators);

};


/**
 *                                                                                                                                                                                                                    
    NET: Command sent from operations console                                                                                                                                                                      
                                                                                                                                                                                                                   
    On entry to all states, state is saved to internal flash                                                                                                                                                       
                                                                                                                                                                                                                   
                                                                                                                                                                                                                          
                                                                                                                                                                                                                      
     NET: Command sent from operations console                                                                                                                                                                        
                                                                                                                                                                                                                      
     On entry to all states, state is saved to internal flash                                                                                                                                                         
                                                                                                                                                                                                                      
                                                                                                                                                                                                                      
                                                                                                                                                                                                                      
                                                                                                                                                                                                                      
                                                                                                                                                                                                                      
                   ┌───────────────────┐                ┌─────────────────────────┐                ┌─────────────────────────┐             ┌─────────────────────────┐                 ┌─────────────────────────┐    
                   │IDLE               │                │ LAUNCH                  │                │ COAST                   │             │ DROGUE                  │                 │ MAIN                    │    
                   │                   │                │                         │                │                         │             │                         │                 │                         │    
    Conneciton     │Entry:             │ NET:Launch     │Entry: Switch to battery,│ T+15s          │Entry: Enable Apogee     │ Apogee      │ Entry:Deploy Drogue     │ Alt<300m        │ Entry: Deploy Main      │    
    ──────────────►│Exit: A            ├───────────────►│Watchdog off             ├───────────────►│detection, Open Vents    ├────────────►│                         ├────────────────►│                         │    
                   │                   │                │                         │                │Exit:                    │             │ Exit:                   │                 │ Exit:                   │    
                   │                   ├────┐           │Exit:                    │                │                         │             │                         │                 │                         │    
                   │                   │    │           │                         │                │                         │             │                         │                 │                         │    
                   └───────────────────┘    │           └───┬─────────────────────┘                └─────────┬───────────────┘             └─────────────────────────┘                 └─────────────────────────┘    
                            ▲               │               │                                                │                                                                                                        
                            │               │               │                                                │                                                                                                        
                            │               │               │                                                │                                                                                                        
                   NET:Idle │      NET:Abort│               │ NET:Abort                                      │                                                                                                        
                            │               │               │                                                │                                                                                                        
                            │               │               │                                                │                                                                                                        
                            │               │               │                                                │                                                                                                        
                            │               │               │                                                │                                                                                                        
                            │               │               │                                                │                                                                                                        
                            │               ▼               ▼                                                │NET:Abort                                                                                               
                            │        ┌────────────────────────────┐                                          │                                                                                                        
                            │        │  ABORT                     │                                          │                                                                                                        
                            └────────┼                            │                                          │                                                                                                        
                                     │ Entry:                     │                                          │                                                                                                        
                                     │ Exit:                      │                                          │                                                                                                        
   Last State IDLE & No Connection   │                            │                                          │                                                                                                        
                         ───────────►│                            │◄─────────────────────────────────────────┘                                                                                                        
                                     │                            │                                                                                                                                                   
                                     │                            │                                                                                                                                                   
                                     │                            │                                                                                                                                                   
                                     │                            │                                                                                                                                                   
                                     └────────────────────────────┘                                                                                                                                                   
                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                   
                                                                                                                                                                                                                   
                                                                                                                                                                                                                   
                                                                                                                                                                                                                   
                                                                                                                                                                                                                   
 * 
 * 
 * 
 * 
 */