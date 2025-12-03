#pragma once


#include "MinosSensor.h"
#include "pins_arduino.h"
#include "BNO055.h"

class BNO {
    public:
        enum channel_config_t {
            IMU_ACCEL_X,
            IMU_ACCEL_Y,
            IMU_ACCEL_Z,
            IMU_ANGVEL_X,
            IMU_ANGVEL_Y,
            IMU_ANGVEL_Z,
            IMU_MAG_X,
            IMU_MAG_Y,
            IMU_MAG_Z,
            IMU_GRAV_X,
            IMU_GRAV_Y,
            IMU_GRAV_Z,
            IMU_LIN_X,
            IMU_LIN_Y,
            IMU_LIN_Z
        };

        bool init(TwoWire &i2c);
        bool update();
        bool IMUinit(TwoWire &i2c);

        
        uint32_t get_data(channel_config_t axis);

        bool getinitialized();

        BNO055_raw_measurment_data_t get_data_struct();

        void set_config(uint8_t *config);

        bool get_config(uint8_t *config);

        uint32_t get_accel_magnitude();
        
    private:
        uint8_t addr = 0x28;
        bool initialized = false;
        BNO055::BNO055_opmode_t mode;
        uint8_t config[22] = {7,0,0,0,160,23,0,32,8,147,201,140,105,211,0,8,88,0,0,0,1,0};
        BNO055_raw_measurment_data_t data;
        uint32_t accel_magnitude;
        
        uint32_t undersample_count;
        const uint32_t num_undersample = 5;
};