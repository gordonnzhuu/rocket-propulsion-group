#include "params.h"
#include "actuators.h"

QueueHandle_t actuator_queue;

void init_actuators() {
    actuator_queue = xQueueCreate(MAX_ACTUATOR_COMMANDS, sizeof(actuator_command_t));
}

void actuator_thread(void* args) {
    while(1) {
        actuator_command_t tmp_cmd;
        while (xQueueReceive(actuator_queue, &tmp_cmd, 0)) {
            tmp_cmd.actuator_driver(tmp_cmd.args);
            //avoid a memory leak with repeated actuator calls
            free(tmp_cmd.args);
        }

        vTaskDelay(ACTUATOR_THREAD_RATE_MS / portTICK_PERIOD_MS);
    }
}

/**********************************
 * PUT ACTUATOR DRIVER FUNCTIONS HERE
 * ********************************/

/*
    EXAMPLE:
    should have return type void and have a single void* param
    define a parameter struct in sensors.h and then cast it in the body of your function
*/

typedef struct {
    int foo;
} actuator_example_args_t; //your arg structs should go in actuators.h

void example_actuator_func(void* args) {
    actuator_example_args_t* s_args = (actuator_example_args_t*) args;
    int foo = s_args->foo; //s_args is a pointer, so we access it with the -> operator

    Serial.println("This does nothing!");
}

void solenoid_driver(void* args) {
    int teensy_pin = -1;
    int solenoid_pin = ((solenoid_args_t*)args)->solenoid_pin;
    bool state = ((solenoid_args_t*)args)->state;

    switch (solenoid_pin) {
        case 1:
            teensy_pin = SOLENOID_1_PIN;
            break;
        case 2:
            teensy_pin = SOLENOID_2_PIN;
            break;
        case 3:
            teensy_pin = SOLENOID_3_PIN;
            break;
        case 4:
            teensy_pin = SOLENOID_4_PIN;
            break;
        case 5:
            teensy_pin = SOLENOID_5_PIN;
            break;
        case 6:
            teensy_pin = SOLENOID_6_PIN;
            break;
        case 7:
            teensy_pin = SOLENOID_7_PIN;
            break;
    }

    if (teensy_pin == -1) {
        return;
    }

    pinMode(teensy_pin, arduino::OUTPUT);   
    digitalWrite(teensy_pin, state);
    Serial.printf("Actuating Solenoid %d to %d\n", solenoid_pin, state);
}

void set_led(void* args) {
    bool led_state = ((led_args_t*)args)->state;
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    digitalWriteFast(arduino::LED_BUILTIN, led_state);
}