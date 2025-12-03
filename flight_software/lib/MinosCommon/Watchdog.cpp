#include "Watchdog.h"

void Watchdog::recv_heartbeat() {
    last_heartbeat_time = millis();
}

void Watchdog::update(ActuatorMap& actuators) {
    if (enabled && millis() - last_heartbeat_time > ttl_ms) {
        for (watchdog_action_t action : action_sequence) {
            actuators[action.pin_num]->set_state(action.state); 
        }
    }
}

void Watchdog::set_watchdog_enabled(bool state){
    enabled = state;
}

bool Watchdog::get_watchdog_enabled(){
    return enabled;
}

void Watchdog::set_watchdog_ttl(uint32_t m_ttl_ms){
    ttl_ms = m_ttl_ms;
}

void Watchdog::add_watchdog_action(uint32_t pin_num, bool state){
    watchdog_action_t action;
    action.pin_num = pin_num;
    action.state = state;

    action_sequence.push_back(action);
}

uint32_t Watchdog::get_watchdog_ttl(){
    return ttl_ms;
}
uint32_t Watchdog::get_offline_time(){
    return millis() - last_heartbeat_time;
}