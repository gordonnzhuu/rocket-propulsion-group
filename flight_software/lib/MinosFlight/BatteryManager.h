#pragma once

#include "minos_util.h"
#include "MinosActuator.h"
#include "Arduino.h"

class BatteryManager : public MinosActuator {
    public:
        BatteryManager(){};

        enum battery_state {
            BATTERY = 1,
            GSE_POWER = 0,
        };

        bool init();
        bool update();

        bool set_state(bool state);
        bool get_state();
        ActuatorEvent get_event();
    private:
        ActuatorEvent event;

        const uint32_t gse_pin = PE2;
        const uint32_t batt_sw_idx = 5; // LOAD_SW_5
        bool current_state;
};
