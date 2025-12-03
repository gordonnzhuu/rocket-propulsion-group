#pragma once

#include "Arduino.h"
#include "minos_util.h"
#include "MinosActuator.h"
#include <vector>

class Watchdog {
    public:
        void update(ActuatorMap& actuators);
        void recv_heartbeat();

        void set_watchdog_enabled(bool state);
        void set_watchdog_ttl(uint32_t m_ttl_ms);
        void add_watchdog_action(uint32_t pin_num, bool state);

        uint32_t get_watchdog_ttl();
        uint32_t get_offline_time();
        bool get_watchdog_enabled();
    private:
        typedef struct {
            uint32_t pin_num;
            bool state;
        } watchdog_action_t;

        bool enabled;
        uint32_t ttl_ms;
        uint32_t last_heartbeat_time;

        std::vector<watchdog_action_t> action_sequence;
};