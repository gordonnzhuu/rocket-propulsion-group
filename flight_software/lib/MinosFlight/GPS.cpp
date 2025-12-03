#include "GPS.h"

bool GPS::init()
{
    bool state = gps.begin(9600);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request RMC and GGA Sentences only
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 5 Hz update rate
    gps.sendCommand(PGCMD_ANTENNA);               // Request for antenna status
    gps.flush();                                  // Clear the buffer
    
    return state;
}

bool GPS::getinitialized()
{
    return initialized;
}

bool GPS::update()
{
    while (gps.available()) gps.read();

    if (gps.newNMEAreceived()) {
        if (!gps.parse(gps.lastNMEA())) {
            return false;
        }
    }
    
    volatile uint8_t ant = gps.antenna;
    if(ant == 3)
    {
        initialized = true;
    }
    else
    {
        initialized = false;
    }
    if (gps.fix) {
        latitude = gps.latitudeDegrees;
        longitude = gps.longitudeDegrees;
        altitude = gps.altitude;
        speed = gps.speed;
        angle = gps.angle;
    }

    antenna_status = gps.antenna;
    fix_status = gps.fix;

    return true;
}

int32_t GPS::get_data(channel_config_t channel)
{
    switch (channel) {
        case GPS_LAT:
            return (int32_t) (latitude * 1000000) ;
        case GPS_LON:
            return (int32_t) (longitude *1000000);
        case GPS_ALT:
            return (int32_t) altitude*1000; // m to mm
        case GPS_SPEED:
            return (int32_t) speed*1000; // m/s to mm/s
        case GPS_ANGLE:
            return (int32_t) angle*1000; // degrees to millidegrees
        case GPS_ANTENNA_STATUS:
            return antenna_status;
        case GPS_FIX_REG:
            return fix_status;
        default:
            return 0.0;
    }
}