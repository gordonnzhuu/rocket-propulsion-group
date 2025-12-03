#include <typeinfo>
#include "StateMachine.h"
#include "internalFlash.h"
#include "Watchdog.h"
#include "BMP388.h"
#include "BNO.h"
#include "ApogeeDetector.h"
#include "SDcard.h"

#define LAUNCH_TIMEOUT 8000
#define COAST_ACCEL 3000 // mm/s^2 this corresponds to 3g acceleration

#ifdef HITL
    #include "HitlManager.h"
#endif

extern Watchdog watchdog;
extern internalFlash iFlash;
extern SDcard sdcard;

extern BNO bno;
extern BMP388 bmp;
extern ApogeeDetector apogee_detector;

#ifdef HITL
    #define LAUNCH_OFFSET 2000
    extern HitlManager hitl;
    bool hitl_en = false;
#endif

bool FlightStateMachine::init(flight_fsm_e init_state){
    change_state(init_state);

    watchdog.add_watchdog_action(0, true); // VOTV
    watchdog.add_watchdog_action(2, true); // VFTV
    watchdog.set_watchdog_ttl(3 * 60 * 1000); // 3 minutes
    watchdog.set_watchdog_enabled(false);

    switch (current_state.state)
    {
    case IDLE: 
        bmp.set_ground_alt();
        break;
    case LAUNCH:
        break;
    case COAST:
        apogee_detector.init(0); // This leads to an immediate apogee detection for some reason
        break;
    case DROGUE:
        break;
    case MAINS:
        break;
    case ABORT:
        watchdog.set_watchdog_enabled(true);
        break;
    default:
        break;
    }
    return true;

}


bool FlightStateMachine::update(SensorMap& sensors, ActuatorMap& actuators) {
    switch (current_state.state)
    {
    case IDLE: 
        // Use commands to change state
        case_idle(sensors, actuators);
        break;
    case LAUNCH:
        case_launch(sensors, actuators);
        break;
    case COAST:
        case_coast(sensors, actuators);
        break;
    case DROGUE:
        case_drogue(sensors, actuators);
        break;
    case MAINS:
        case_mains(sensors, actuators);
        break;
    case ABORT:
        case_abort(sensors, actuators);
        break;
    default:
        break;
    }
    return true;
}

bool FlightStateMachine::net_command(flight_fsm_e command) {
    net_cmd_state = command;
    return true;
}

bool FlightStateMachine::change_state(flight_fsm_e new_state) {
    current_state.state  = new_state;
    current_state.init_time = millis();
    net_cmd_state = new_state; // Set net_cmd_state to new state so that it does not loop indefinitely between states

    volatile uint32_t time = millis();
   // iFlash.setFlightState(new_state);
    volatile uint32_t dt1 = millis() - time;
    sdcard.writeState(current_state);
    volatile uint32_t dt2 = millis() - time;

    __NOP();
    return true;
}

bool FlightStateMachine::case_idle(SensorMap& sensors, ActuatorMap& actuators) {

    if(net_cmd_state == ABORT){
        change_state(ABORT);
    } else if (net_cmd_state == LAUNCH) {
        // Watchdog off
        watchdog.set_watchdog_enabled(false);
        change_state(LAUNCH);
    }
    return true;
}

bool FlightStateMachine::case_launch(SensorMap& sensors, ActuatorMap& actuators) {

    apogee_detector.add_measurement(
        bmp.get_data(BMP388::BMP_ALT)
    );

    #ifdef HITL
        if (millis() - current_state.init_time >= LAUNCH_OFFSET && !hitl_en ){
            hitl.set_hitl_mode(HitlManager::MODE_REAL_TIME);
            hitl_en = true;
        }
    #endif

    if(net_cmd_state == ABORT){
        watchdog.set_watchdog_enabled(true);
        change_state(ABORT);
    // // Oring the accel magnitude with the time out does not work this way because on the pad, the accel will be less than 10000 so this went to coast immediately, oops
    //} else if (millis() - current_state.init_time >= LAUNCH_TIMEOUT || bno.get_accel_magnitude() < 10000){ 
    } else if (millis() - current_state.init_time >= LAUNCH_TIMEOUT){ 

        apogee_detector.init(-9.8); // Start testing for apogee!
        change_state(COAST);
    }
    


    return true;
}

bool FlightStateMachine::case_coast(SensorMap& sensors, ActuatorMap& actuators) {
    apogee_detector.add_measurement(
        bmp.get_data(BMP388::BMP_ALT)
    );
    apogee_detector.update();

    if(net_cmd_state == ABORT){
        watchdog.set_watchdog_enabled(true);
        change_state(ABORT);
    } else if(apogee_detector.poll_apogee()){
        // FIRE DROGUES
        actuators[6]->set_state(true); // Fire PYRO 1
        
        change_state(DROGUE);
    }
    return true;
}

bool FlightStateMachine::case_drogue(SensorMap& sensors, ActuatorMap& actuators) {

    apogee_detector.add_measurement(
        bmp.get_data(BMP388::BMP_ALT)
    );
    apogee_detector.update();


    if(bmp.get_data(BMP388::BMP_ALT) < MAIN_DEPLOY_ALT){
        // FIRE MAINS
        actuators[7]->set_state(true); // Fire PYRO 2
        change_state(MAINS);
    }

    if(millis() - current_state.init_time >= 2000 && actuators[6]->get_state() == true)
    {
        actuators[6]->set_state(false); // Close PYRO 1
    }

    // Open vents some time into coast to make sure that there is no pressure 
    if(millis() - current_state.init_time >= 10000 && actuators[0]->get_state() == false){
        // open vents
        actuators[0]->set_state(true); // Open VOTV
        actuators[2]->set_state(true); // Open VFTV
    }
    return true;
}

bool FlightStateMachine::case_mains(SensorMap& sensors, ActuatorMap& actuators) {

    apogee_detector.add_measurement(
        bmp.get_data(BMP388::BMP_ALT)
    );
    apogee_detector.update();

     if(millis() - current_state.init_time >= 2000 && actuators[7]->get_state() == true)
    {
        actuators[7]->set_state(false); // Close PYRO 2
    }


    // To reset the system after the flight /  HITL
    if(net_cmd_state == IDLE){
        change_state(IDLE);
    }

    return true;
}

bool FlightStateMachine::case_abort(SensorMap& sensors, ActuatorMap& actuators) {

    if(net_cmd_state == IDLE){
        change_state(IDLE);
    }

    return true;
}

