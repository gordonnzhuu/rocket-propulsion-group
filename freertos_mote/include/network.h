#ifndef NETWORK_H
#define NETWORK_H

#include "queue.h"

//const macros
#define UDP_PORT 8888
#define MOTE1_IP ip(192, 168, 1, 101)
#define TEMP_MAC { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}
#define PACKET_BUFFER_SIZE 64

//function declarations
void UDP_Thread(void* arg);
void init_UDP();

//static variable declarations
extern QueueHandle_t telemetry_queue;

//typedefs
typedef struct {
    int pin_num;
    int data;
} sensor_telemetry_t;

typedef struct {
    uint8_t pin_num;
    uint8_t data_bytes[4];
} legacy_telemetry_packet_t;

//sensor constants
typedef enum {
    TeensyADC=1,
    I2C_ADC_1ch=2,
    I2C_ADC_2ch=3,
    Flowmeter=4, //unused
    I2C_ADC_2ch_PGA2=7,
    I2C_ADC_2ch_PGA4=8,
    I2C_ADC_2ch_PGA8=9,
    I2C_ADC_2ch_PGA16=10,
    I2C_ADC_2ch_PGA32=11,
    I2C_ADC_2ch_PGA64=12,
    I2C_ADC_2ch_PGA128=13,
    ADC_INTERNAL_TEMP=14
} sensor_driver_enum;

#endif