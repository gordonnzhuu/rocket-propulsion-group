#if defined(MOTE4_TEENSY41) || defined(MOTE5_TEENSY41)
    #include "arduino_freertos.h"
    #include "queue.h"
    #include "avr/pgmspace.h"

    #include <NativeEthernet.h>
#elif defined(MOTE_STM32)
    #include <STM32FreeRTOS.h>
    #include <STM32Ethernet.h>
#endif

#include <vector>
#include "params.h"
#include "network.h"
#include "sensors.h"
#include "actuators.h"

byte gateway[] = { 255, 255, 255, 0} ;
byte netmask[] = { 255, 255, 255, 0} ;
IPAddress MOTE1_IP;
byte mac[] = TEMP_MAC;
EthernetUDP Udp;

QueueHandle_t telemetry_queue;

//note: function should NOT be called in a thread
void init_UDP() {
    Serial.println("Calling Ethernet-Begin");
    Ethernet.begin(mac, ip);
    Serial.println("Called Ethernet-Begin");

    telemetry_queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_telemetry_t));

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        while (1) {
            Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
            delay(10); // do nothing, no point running without Ethernet hardware
        }
    }

    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
    }

    // start UDP
    Udp.begin(UDP_PORT);

    Serial.println("Sucess creating UDP connection!");
}

void parse_legacy_packet(char* data ) {
    uint8_t pin_num = data[0];
    uint8_t actuator_write_command = (data[1] & 0b10000000) == 0b10000000;
    uint8_t actuator_state = (data[1] & 0b01000000) == 0b01000000;
    uint8_t interface_type_number = (data[1] & 0b00111111);

    Serial.println("Recieved packet!");

    if (actuator_write_command) {
        actuator_command_t new_cmd;

        switch(interface_type_number) {
            case SOLENOID: {
                //set the solenoid driver
                new_cmd.actuator_driver = &solenoid_driver;
                //make sure the args don't get cleared with stack memory
                solenoid_args_t* args = (solenoid_args_t*)malloc(sizeof(solenoid_args_t));

                args->solenoid_pin = pin_num;
                args->state = actuator_state;
                new_cmd.args = args;

                xQueueSend(actuator_queue, &new_cmd, 0);
                break;
            }
            case LED: {
                new_cmd.actuator_driver = &set_led;
                solenoid_args_t* args = (solenoid_args_t*)malloc(sizeof(solenoid_args_t));
                args->state = actuator_state;
                new_cmd.args = args;

                xQueueSend(actuator_queue, &new_cmd, 0);
                break;
            }
        }
    } else {
        Serial.println("Recieved sensor packet!");

        sensor_handle_t new_handle;
        new_handle.pin_num = pin_num;

        switch (interface_type_number) {
            case TeensyADC:
                 //internal ADC setup
                //pinMode(TEENSY_INTERNAL_ADC_PIN, INPUT);
                new_handle.sensor_driver = &read_teensy_adc;
                //note: 0 in this array indicates bus #0. Future code will need better logic to decide which bus to send to
                xQueueSend(sensor_thread_list[0].queue, &new_handle, 0);
                Serial.println("Added sensor to queue!");
                break;
            case I2C_ADC_1ch:
                break;
            //TODO add more cases
        }
    }
}

legacy_telemetry_packet_t write_to_legacy_packet(uint8_t pin_num, int32_t data) {
    legacy_telemetry_packet_t packet;
    packet.pin_num = pin_num;
    packet.data_bytes[0] = data & 0x000000ff;
    packet.data_bytes[1] = (data & 0x0000ff00) >> 8;
    packet.data_bytes[2] = (data & 0x00ff0000) >> 16;
    packet.data_bytes[3] = (data & 0xff000000) >> 24;

    return packet;
}

void UDP_Thread(void* arg) {
    while(1){
        IPAddress remote;
        int packetSize = Udp.parsePacket();
        char packetBuffer[PACKET_BUFFER_SIZE];
        
        if (packetSize) {
            remote = Udp.remoteIP();
            Udp.read(packetBuffer, PACKET_BUFFER_SIZE);

            Serial.println("Recv packet!");
            
            parse_legacy_packet(packetBuffer);
        }
        
        //send data packets back to client
        legacy_telemetry_packet_t telemetry_to_send[64];
        sensor_telemetry_t tmp_packet;
        int num_telem_packets = 0;
        
        while(xQueueReceive(telemetry_queue, &tmp_packet, 0)) {
            Serial.println("Recv telem!");
            telemetry_to_send[num_telem_packets] = write_to_legacy_packet(tmp_packet.pin_num, tmp_packet.data);
            num_telem_packets++;
        }
        
        if (num_telem_packets > 0){
            Udp.beginPacket(Udp.remoteIP(), UDP_PORT);
            Udp.write((uint8_t*)telemetry_to_send, num_telem_packets*sizeof(legacy_telemetry_packet_t));
            Udp.endPacket();
        }

        //Serial.println("Looping network thread!");
        vTaskDelay(NETWORK_THREAD_RATE_MS / portTICK_PERIOD_MS); //wait 50 ms
    }
}