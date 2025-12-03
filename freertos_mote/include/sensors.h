#ifndef SENSORS_H
#define SENSORS_H

#include "params.h"
#if defined(MOTE4_TEENSY41) || defined(MOTE5_TEENSY41)
    #include "arduino_freertos.h"
    #include "semphr.h"
    #include "queue.h"
#elif defined(MOTE_STM32)
    #include <STM32FreeRTOS.h>
#endif

#include <SPI.h>

typedef struct {
    //handle of the thread itself
    portBASE_TYPE handle;
    int bus_num;
    //mutex for any critical resources this bus accesses
    SemaphoreHandle_t mutex;
    //write new sensors to this queue. 
    //If there needs to be more data passed to sensor threads in future, we can update this system
    QueueHandle_t queue;
} sensor_thread_t;

//The **formulas** used to convert from AD7193 output codes to voltages appear on **page 33** of the **AD7193 data sheet** (Rev. E was the most recent ver. at the time of writing)!
typedef struct {
    int bus_num;
} sensor_thread_arg_t;

typedef int (*sensor_function_t)(void*);

typedef struct {
    uint8_t pin_num;
    void* sensor_args;
    //function pointer to the function called to make the sensor readings. Should be assumed to be thread safe.
    //the argument is a single void pointer for now, might change later to a more userproof set of args
    sensor_function_t sensor_driver;
} sensor_handle_t;

extern sensor_thread_t sensor_thread_list[];

void sensor_thread(void* args);

/***********************************
 * Put ARGS structs for sensors here
 ***********************************/

//Macros relevant to the Communications register... (the first byte transferred to the AD7193 in a transaction will always be written to the Comm register, so it has no unique address)
//Bits 0, 6, and 7 must always be empty (0). Bit 1 = r/w ; bits 2-4 = register address ; bit 5 = enable continuous read (has an effect for data reg reads only)
#define AD7193_REG_READ 0b01000000            //Signals a read operation
#define AD7193_REG_WRITE 0b00000000           //Signals a write operation
#define AD7193_STATUS_REG 0b00000000        //8-bit reg (r)
#define AD7193_MODE_REG 0b00001000          //24-bit reg (r/w)  
#define AD7193_CONFIG_REG 0b00010000        //24-bit reg (r/w)
#define AD7193_DATA_REG 0b00011000          //24-bit reg (r)
#define AD7193_ID_REG 0b00100000            //8-bit reg (r)
#define AD7193_GPOCON_REG 0b00101000        //8-bit reg (r/w)
#define AD7193_OFFSET_REG 0b00110000        //24-bit reg (r/w)
#define AD7193_FULLSCALE_REG 0b00111000     //24-bit reg (r/w)

#define AD7193_CHANNEL_BIT 8
#define AD7193_DIFFERENTIAL_BIT 18
#define AD7193_PGA_BIT 0
#define AD7183 POLARITY_BIT 3
#define AD7193_CHANNEL_SHIFT 0x00000100

//Have to include some global variables here that affect the conversion from output code to volts for the AD7193 --
//change these in the AD7193 functions or hardcode them to something if they're never going to change!!!

//This info appears on sheet 3 of the MoteV5 schematic -- it appears as REFIN1(+)!!
#define VREF 5
#define AD7193_CODE_EXP_TERM 16777216   //This is 2^24

#define TEENSY_INTERNAL_ADC_PIN 14

typedef struct {
    uint8_t comm_bits;
    uint32_t write_payload;
    SPIClass* spi_bus;      //To fill this field, please use either "&SPI" (for AD7193 1) or "&SPI1" (for AD7193 2)!!!
} AD7193_arg_t;

typedef struct {
    //Can add more config fields to this if necessary.
    //The channel settings are 10 bits occupying the range [17:8] within the config register, so any valid value for channel_settings must use no more than the 10 LSBs of the word!
    //Also, it will need to be left shifted by 8 bits before it can be safely inserted into a config write payload.
    bool internal_temp; //if this bool is TRUE, all other parameters will be ignored

    bool differential;
    uint16_t spi_num; //should be 0 or 1
    uint16_t channel;
    uint16_t pga_gain;
} AD7193_wrapper_arg_t;

int read_teensy_adc(void* args);

int AD7193_driver(void* args);
int AD7193_wrapper(void* args);
int AD7193_codeToVolts(uint32_t code);

int example_sensor_func(void* args);

#endif