#pragma once

#include <BMP388_DEV.h>
#include "MinosSensor.h"

// Friends of Amateuer rocketry Altitude in meters
// Default Altitude
#define FAR_ALT 621
class BMP388  {
    
    public:
        enum channel_config_t {
            BMP_ALT,
            BMP_TMP,
            BMP_P
        };

        BMP388(TwoWire &i2c):
            bmp388(i2c) 
            {};
        
        bool init();
        bool update();
        bool getinitialized();
        
        int32_t get_data(channel_config_t channel);

        void set_seaLevel(float seaLevelPressure);
        float get_seaLevel();
        void set_ground_alt();

    private:
        uint8_t addr = 0x77;
        bool initialized = false;
        float altitude;
        float temperature;
        float pressure;
        float seaLevelPressure = 1015.24;
        BMP388_DEV bmp388;

        float ground_altitude = FAR_ALT; 
        bool ground_alt_set = true;
};