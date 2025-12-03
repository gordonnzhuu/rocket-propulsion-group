#include "BatteryManager.h"

extern ActuatorMap actuators;

bool BatteryManager::init() {
    pinMode(gse_pin, OUTPUT);

    digitalWrite(gse_pin,false);
    actuators[batt_sw_idx]->set_state(false);

    return true;
}

bool BatteryManager::update() {
    return true;
}

bool BatteryManager::get_state() {
    return current_state;
}

bool BatteryManager::set_state(bool state) {
    current_state = state;

    event.state = current_state;
    event.time = millis();

    if (state == BATTERY) {
        // Make
        actuators[batt_sw_idx]->set_state(false);
        // Wait
        delay(2);
        // Break
        digitalWrite(gse_pin,false);
    } else if (state == GSE_POWER) {
        // Make
        digitalWrite(gse_pin,true);
        // Wait
        delay(2);
        // Break
        actuators[batt_sw_idx]->set_state(true);
    }

    return true;
}

ActuatorEvent BatteryManager::get_event() {
    return event;
}