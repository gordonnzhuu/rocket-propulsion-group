#pragma once 

#include "Adafruit_GPS.h"
#include <HardwareSerial.h>
#include "MinosSensor.h"

class GPS {
    public: 
        enum channel_config_t {
            GPS_LAT,
            GPS_LON,
            GPS_ALT,
            GPS_SPEED,
            GPS_ANGLE,
            GPS_ANTENNA_STATUS,
            GPS_FIX_REG,
            GPS_FIX_LED,
        };

        GPS(HardwareSerial &serial):
            gps(&serial) 
            {};

        bool init();
        bool update();
        bool getinitialized();

        int32_t get_data(channel_config_t channel);

    private:
        Adafruit_GPS gps;
        bool initialized = false;
        float latitude;
        float longitude;
        float altitude;
        float speed;
        float angle;

        // Status values
        uint8_t antenna_status;
        bool fix_status;
};