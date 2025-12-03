/**
 * This file defines a table where, for each channel, it maps the channel type to the number of bits and scale it should use
 */
#include <stdlib.h>
#include "Arduino.h"
#include "minos_util.h"

struct rf_param_t {
    uint8_t bits = 32;
    float scale = 1;
};

static std::map<uint8_t, rf_param_t> loraMap = {
    {TEENSY_ADC, rf_param_t()},
    {I2C_ADC_1CH, {13, 1}},
    {I2C_ADC_2CH, rf_param_t()},
    {FLOWMETER, rf_param_t()},
    {I2C_ADC_2CH_MIN_GAIN, rf_param_t()},
    {I2C_ADC_2CH_MAX_GAIN, rf_param_t()},
    {INTERNAL_TEMP, {8, 1}},
    {SPI_ADC_1CH, {14, 1e3}},
    {SPI_ADC_2CH, rf_param_t()},
    {SPI_ADC_2CH_MIN_GAIN, rf_param_t()},
    {SPI_ADC_2CH_MAX_GAIN, rf_param_t()},
    {BMP_ALT, {16, 3e4}},
    {IMU, {16, 1}},
    {VOLT_MONITOR, {11, 10}},
    {LOOP_TIMER, {8, 1e3}},
    {BMP_TEMP, {8, 1e3}},
    {BMP_PRESSURE, rf_param_t()},
    {GNC_ALT_VEL, {10, 1e3}},
    {GNC_STATE, {5, 1}},
    {GPS_SENSORS, {28, 1}},
};