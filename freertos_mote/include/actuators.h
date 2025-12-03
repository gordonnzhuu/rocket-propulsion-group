#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "params.h"
#if defined(MOTE4_TEENSY41) || defined(MOTE5_TEENSY41)
    #include "arduino_freertos.h"
    #include "avr/pgmspace.h"
    #include "queue.h"
#elif defined(MOTE_STM32)
    #include <STM32FreeRTOS.h>
#endif

#define MAX_ACTUATOR_COMMANDS 32

extern QueueHandle_t actuator_queue;

void init_actuators();
void actuator_thread(void* args);

typedef enum {
    LED = 45,
    SOLENOID = 6,
} actuator_type_t;

typedef void (*actuator_function_t)(void*);

typedef struct {
    actuator_function_t actuator_driver;
    void* args;
} actuator_command_t;

/* **********************
 * ACTUATOR DRIVER FUNCTIONS AND ARGS
 * *********************/
#define SOLENOID_1_PIN 5
#define SOLENOID_2_PIN 6
#define SOLENOID_3_PIN 7
#define SOLENOID_4_PIN 8
#define SOLENOID_5_PIN 9
#define SOLENOID_6_PIN 29
#define SOLENOID_7_PIN 28

typedef struct {
    int solenoid_pin;
    bool state;
} solenoid_args_t;

void solenoid_driver(void* args);

//LED for debugging
typedef struct {
    bool state;
} led_args_t;
void set_led(void* args);

#endif