#pragma once
#include "MinosSensor.h"
#include "MinosActuator.h"
#include <map>
#include <memory>
#include <stdint.h>

/**
 * Pile up typedefs, util functions, etc. here
*/
typedef std::map<uint32_t, MinosSensor*> SensorMap;
typedef std::map<uint32_t, MinosActuator*> ActuatorMap;

typedef std::map<uint8_t, uint8_t> SensorSerialMap;

/**
 * Device type enums.
 * 
 * Legacy bullshit.
 * Handle with care.
*/

// Sensor constants
typedef enum {
    TEENSY_ADC = 1,
    I2C_ADC_1CH,
    I2C_ADC_2CH,
    FLOWMETER,
    I2C_ADC_2CH_MIN_GAIN = 7,
    I2C_ADC_2CH_MAX_GAIN = 13,
    INTERNAL_TEMP,
    SPI_ADC_1CH,
    SPI_ADC_2CH,
    SPI_ADC_2CH_MIN_GAIN,
    SPI_ADC_2CH_MAX_GAIN = 23,
    BMP_ALT,
    IMU,
    VOLT_MONITOR,
    LOOP_TIMER,
    BMP_TEMP,
    BMP_PRESSURE,
    GNC_ALT_VEL,
    GNC_STATE,  
    GPS_SENSORS=33,
    SENSOR_CLEAR_CODE = 44

} sensor_interface_num_t;

// Actuator constants
typedef enum {
    PWM_SERVO = 5,
    BINARY_LOAD_SWITCH,
    LAUNCH_STATE,
    IDLE_STATE,
    ABORT_STATE,
    LORA_PWR,
    HITL_ASSERT,
    BATTERY_MANAGER = 32,
    BANG_BANG_CONTROL = 41,
    AUTO_FIREX_CONTROL,
    HEARTBEAT,
    WATCHDOG_TOGGLE = 46
} actuator_interface_num_t;

// MoTE packet type
typedef struct {
    uint8_t pin_num;
    uint8_t data_bytes[4];
} mote_packet_t;

