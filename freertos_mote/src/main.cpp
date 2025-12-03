/*
 * Example to demonstrate thread definition, semaphores, and thread sleep.
 */
#if defined(MOTE4_TEENSY41) || defined(MOTE5_TEENSY41)
    #include "arduino_freertos.h"
    #include "queue.h"
    #include "semphr.h"
    #include "avr/pgmspace.h"

    #include <NativeEthernet.h>
    #include <NativeEthernetUdp.h>
#elif defined(MOTE_STM32)
    #include <STM32FreeRTOS.h>
#endif

#include "params.h"
#include "sensors.h"
#include "network.h"
#include "actuators.h"

sensor_thread_t sensor_thread_list[NUM_SENSOR_BUSES];

void setup() {
  Serial.begin(9600);
  
  //initialize UDP stuff

  Serial.println("Beginning Setup");
  init_UDP();
  Serial.println("UDP Setup complete!");

  portBASE_TYPE network_thread = xTaskCreate(&UDP_Thread, NULL, DEFAULT_STACK_SIZE, NULL, 1, NULL);
  Serial.println("Began network thread!");

  init_actuators();
  portBASE_TYPE actuator_thread_handle = xTaskCreate(&actuator_thread, NULL, DEFAULT_STACK_SIZE, NULL, 1, NULL);
  Serial.println("Began actuator thread");

  //initialize sensor threads
  for (int i = 0; i < NUM_SENSOR_BUSES; i++) {
    sensor_thread_arg_t* tmp_sensor_thr_args_ptr = (sensor_thread_arg_t*)malloc(sizeof(sensor_thread_arg_t));
    tmp_sensor_thr_args_ptr->bus_num = i;
    Serial.printf("Pointer to ARGS [main]: %p\n", tmp_sensor_thr_args_ptr);
    
    sensor_thread_list[i].bus_num = i;
    sensor_thread_list[i].handle = xTaskCreate(&sensor_thread, NULL, DEFAULT_STACK_SIZE, tmp_sensor_thr_args_ptr, 2, NULL);
    sensor_thread_list[i].mutex = xSemaphoreCreateMutex();
    sensor_thread_list[i].queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_handle_t));
    Serial.printf("Pointer to Q [main]: %p\n", sensor_thread_list[i].queue);
  }
  Serial.println("Began sensor threads!");

  //initialize main actuator thread
  ; //TODO

  // start scheduler
  vTaskStartScheduler();
}

//do not use LOOP for RTOS apps
void loop() {}

