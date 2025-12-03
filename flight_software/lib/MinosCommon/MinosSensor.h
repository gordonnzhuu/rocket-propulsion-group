#pragma once

#include "Arduino.h"
#include <cstdint>

#define HITL_TABLE 0

struct SensorReading {
    uint32_t data;
};

struct BNOReading : SensorReading {
    uint32_t data0;
    uint32_t data1;
    uint32_t data;
};

struct rf_config_t {
    bool enable;
    uint8_t num_bits = 32;
    float scale = 1;
};

class MinosSensor
{
    
    public:
        virtual bool init() {
            return false;
        }

        virtual bool update() {return false;}
        virtual int32_t get_data() {return 0;}
        
        virtual void enable_rf(bool enable) {
            rf_config.enable = enable;
        }

        virtual void set_rf_config(bool enable, uint8_t num_bits, float scale = 1) {
            rf_config.enable = enable;
            rf_config.num_bits = num_bits;
            rf_config.scale = scale;
        };
        
        virtual void set_rf_config(rf_config_t config) {
            rf_config = config;
        };

        rf_config_t get_rf_config() {
            return rf_config;
        };

    private:
        rf_config_t rf_config;
};

