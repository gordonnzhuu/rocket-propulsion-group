#include "SPI.h"
#include "network.h"
#include "AD7193.h"
#include "AdcChannel.h"
#include "SDcard.h"
#include "LoopTimer.h"
#include "Watchdog.h"
#include <vector>

#ifdef FLIGHT_COMPUTER
	#include "EthernetENC.h"
	#include <EthernetUdp.h>
    #include "BMP388.h"
    #include "BMPChannel.h"
    #include "BNO.h"
    #include "BNOChannel.h"
    #include "VoltMonitor.h"
    #include "StateMachine.h"
    #include "AlphaBetaFilter.h"
    #include "GNCChannel.h"
    #include "StateChannel.h"
    #include "internalFlash.h" 
    #include "GPS.h"
    #include "GPSChannel.h"
#else
    #include <NativeEthernet.h>    
    #include <NativeEthernetUdp.h>
#endif

#ifdef BUILD_LORA
    #include "Lora.h"
    #include "LoraMap.h"
#endif

// 9E-20-62-5A-B7-FB
byte mac_addr[] = {0x9E, 0x20, 0x62, 0x5A, 0xB7, 0xFB};

IPAddress ip;
unsigned int localPort = 8888; // local port to listen on

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
IPAddress remoteIP;
std::vector<mote_packet_t> ack_q;

uint32_t commands_recv = 0;
uint32_t commands_acked = 0;

//Hardware ADC sensor instances
extern AD7193 Adc0;
extern AD7193 Adc1;
extern SDcard sdcard;
extern Watchdog watchdog;
extern SensorSerialMap sensors_serial;

#ifdef FLIGHT_COMPUTER
    extern TwoWire sensor_i2c;
    extern FlightStateMachine state_machine;
    extern internalFlash iFlash;
#endif

#ifdef BUILD_LORA
  extern Lora lora;
#endif


#ifndef FLIGHT_COMPUTER
void teensyMAC(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
#endif

mote_packet_t write_to_mote_packet(uint8_t pin_num, int32_t data) {
    mote_packet_t packet;
    packet.pin_num = pin_num;
    packet.data_bytes[0] = data & 0x000000ff;
    packet.data_bytes[1] = (data & 0x0000ff00) >> 8;
    packet.data_bytes[2] = (data & 0x00ff0000) >> 16;
    packet.data_bytes[3] = (data & 0xff000000) >> 24;

    return packet;
}



/**
 * Set up a UDP connection using mote-style network.
 * @return Whether the network was sucessfully created.
*/
bool init_motenet()
{
    /**
     * Configure SPI to use the correct pins.
     * @note The convention will be that the global SPI object is for ethernet, unless we change the library to allow using a SPIClass.
    */
    #ifdef FLIGHT_COMPUTER
        SPI.setMOSI(PA7);
        SPI.setMISO(PA6);
        SPI.setSCLK(PA5);

        Ethernet.init(PA4);
    #endif

    if(!sdcard.getIPconfig(&ip)) {
        return false;
    }

    #ifndef FLIGHT_COMPUTER
        // if not flightcomputer, overwrite mac address
        // if flight computer, hardcode mac address. There is at most one FC in system at a time so its fine.
        teensyMAC(mac_addr);
    #endif
    
    Ethernet.begin(mac_addr, ip);
    
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("No ETH Hardware");
        return false;
    }

    delay(500);

    //Check for link status.
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("No ETH LINK");
        return false;
    }

    // start UDP
    Udp.begin(localPort);
    return true;
}

void queue_ack(uint32_t pin_num, bool actuator_state) {
    ack_q.push_back(
        write_to_mote_packet(pin_num + 100, (int32_t) actuator_state)
    );
    commands_recv++;
}

/**
 * Adds sensors, this is seperate function since the flash also needs to add sensors
 * @param sensors The sensor map to write to.
*/
bool add_sensors(uint8_t pin_num, uint8_t enable_rf, uint8_t interface_type_number, SensorMap& sensors){
    volatile uint8_t tmp = interface_type_number;
    
    if (interface_type_number == SPI_ADC_1CH) {
            pin_num / 10 == 0 ? Adc0.set_differential(false) : Adc1.set_differential(false);
            pin_num / 10 == 0 ? Adc0.set_pga_gain(1) : Adc1.set_pga_gain(1);

            sensors[pin_num] = new AdcChannel(
                pin_num % 10, //channel
                pin_num / 10 == 0 ? Adc0 : Adc1
            );
            sensors[pin_num]->init();
        } else if (interface_type_number >= SPI_ADC_2CH && interface_type_number <= SPI_ADC_2CH_MAX_GAIN) {
            pin_num / 10 == 0 ? Adc0.set_differential(true) : Adc1.set_differential(true);
            
            uint16_t pga = (uint16_t)pow(2, interface_type_number - SPI_ADC_2CH);
            pin_num / 10 == 0 ? Adc0.set_pga_gain(pga) : Adc1.set_pga_gain(pga);

            sensors[pin_num] = new AdcChannel(
                pin_num % 10, //channel
                pin_num / 10 == 0 ? Adc0 : Adc1
            );
            sensors[pin_num]->init();
        } else if (interface_type_number == INTERNAL_TEMP) {
            sensors[pin_num] = new AdcChannel(
                8,
                pin_num / 10 == 0 ? Adc0 : Adc1
            );
            sensors[pin_num]->init();
        } else if (interface_type_number == LOOP_TIMER) {
                //loop timer is always on pin 80, just because
                sensors[80] = new LoopTimer(0.1);
                sensors[80]->set_rf_config(enable_rf, 13, 1e-3);
                sensors[80]->init();
        }
        #ifdef FLIGHT_COMPUTER
            else if (interface_type_number == BMP_ALT) {
                BMP388::channel_config_t channel = BMP388::BMP_ALT;

                sensors[pin_num] = new BMPChannel(channel);
                sensors[pin_num]->init();
            }else if (interface_type_number == BMP_PRESSURE) {
                BMP388::channel_config_t channel = BMP388::BMP_P;
                sensors[pin_num] = new BMPChannel(channel);
                sensors[pin_num]->init();
            } else if (interface_type_number == BMP_TEMP) {
                BMP388::channel_config_t channel = BMP388::BMP_TMP;
                sensors[pin_num] = new BMPChannel(channel);
                sensors[pin_num]->init();
            } else if (interface_type_number == GNC_ALT_VEL) {
                sensors[pin_num] = new GNCChannel(GNCChannel::ALT_VEL);
                sensors[pin_num]->init();
            } else if (interface_type_number == GNC_STATE) {
                sensors[pin_num] = new StateChannel();
                sensors[pin_num]->init();
            } else if (interface_type_number == IMU) {
                BNO::channel_config_t channel;

                if      (pin_num == 50) channel = BNO::IMU_ACCEL_X;
                else if (pin_num == 51) channel = BNO::IMU_ACCEL_Y;
                else if (pin_num == 52) channel = BNO::IMU_ACCEL_Z;
                else if (pin_num == 53) channel = BNO::IMU_ANGVEL_X;
                else if (pin_num == 54) channel = BNO::IMU_ANGVEL_X;
                else if (pin_num == 55) channel = BNO::IMU_ANGVEL_X;
                else if (pin_num == 56) channel = BNO::IMU_MAG_X;
                else if (pin_num == 57) channel = BNO::IMU_MAG_Y;
                else if (pin_num == 58) channel = BNO::IMU_MAG_Z;
                else if (pin_num == 59) channel = BNO::IMU_GRAV_X;
                else if (pin_num == 60) channel = BNO::IMU_GRAV_Y;
                else if (pin_num == 61) channel = BNO::IMU_GRAV_Z;
                else if (pin_num == 62) channel = BNO::IMU_LIN_X;
                else if (pin_num == 63) channel = BNO::IMU_LIN_Y;
                else if (pin_num == 64) channel = BNO::IMU_LIN_Z;

                sensors[pin_num] = new BNOChannel(channel);
            } else if (interface_type_number == VOLT_MONITOR) {
                uint32_t io_pin;
                if      (pin_num == 71) io_pin = PC1; // 12V
                else if (pin_num == 72) io_pin = PC2; // 3V3
                else if (pin_num == 73) io_pin = PC3; // 5V
                else                    return false;

                sensors[pin_num] = new VoltMonitor(io_pin);
                sensors[pin_num]->init();
            } else if (interface_type_number == GPS_SENSORS) {
                GPS::channel_config_t channel;
                if (pin_num == 90) channel = GPS::GPS_LAT;  
                else if (pin_num == 91) channel = GPS::GPS_LON;
                else if (pin_num == 92) channel = GPS::GPS_ALT;
                else if (pin_num == 93) channel = GPS::GPS_SPEED;
                else if (pin_num == 94) channel = GPS::GPS_ANGLE;
                else if (pin_num == 95) channel = GPS::GPS_ANTENNA_STATUS;
                else if (pin_num == 96) channel = GPS::GPS_FIX_REG;

                sensors[pin_num] = new GPSChannel(channel);
                sensors[pin_num]->init();
            }
        #endif
        else if (interface_type_number == SENSOR_CLEAR_CODE) {
            sensors.clear();
            sensors_serial.clear();
            return true;
        }
        #ifdef FLIGHT_COMPUTER
        if (loraMap.count(interface_type_number) > 0) {
            sensors[pin_num]->set_rf_config(enable_rf, loraMap[interface_type_number].bits, loraMap[interface_type_number].scale);
        } else {
            __NOP();
        }
        #endif
        return true;
    }

bool parse_mote_packet(char* data, SensorMap& sensors, ActuatorMap& actuators) {
    uint8_t pin_num = data[0];
    uint8_t actuator_write_command = (data[1] & 0b10000000) == 0b10000000;
    // actuator_state and enable_rf are overloaded, since enable_rf is meaningless for actuators
    uint8_t actuator_state = (data[1] & 0b01000000) == 0b01000000;
    uint8_t enable_rf = (data[1] & 0b01000000) == 0b01000000;
    uint8_t interface_type_number = (data[1] & 0b00111111);

    if (actuator_write_command) {
        if (interface_type_number == BINARY_LOAD_SWITCH) {
            //actuators[PNID]->init(pin_num)
            if (actuators.count(pin_num) != 0) {
                actuators[pin_num]->set_state(actuator_state);
                queue_ack(pin_num, actuator_state);
            }
        } else if (interface_type_number == WATCHDOG_TOGGLE) {
            watchdog.set_watchdog_enabled(actuator_state);
            queue_ack(pin_num, actuator_state);
        } 
#ifdef FLIGHT_COMPUTER
        else if (interface_type_number == LAUNCH_STATE) {
            // Only acknowledge when going from IDLE to LAUNCH, or LAUNCH to itself
            // State  machine will not go from ABORT to LAUNCH so don't acknowledge that
            if((state_machine.get_state() == IDLE) || (state_machine.get_state() == LAUNCH)){

                if (actuator_state)
                    state_machine.net_command(LAUNCH);
                queue_ack(pin_num, actuator_state);

            }
        } else if (interface_type_number == IDLE_STATE) {
            if (actuator_state)
                state_machine.net_command(IDLE);
            queue_ack(pin_num, actuator_state);
        } else if (interface_type_number == ABORT_STATE) {
            if (actuator_state)
                state_machine.net_command(ABORT);
            queue_ack(pin_num, actuator_state);
        } else if (interface_type_number == BATTERY_MANAGER) {
            actuators[30]->set_state(actuator_state);
            queue_ack(pin_num, actuator_state);
        } 
        else if (interface_type_number == HITL_ASSERT) {
            #ifdef HITL
                if (actuator_state == true) queue_ack(pin_num, actuator_state);
            #else
                if (actuator_state == false) queue_ack(pin_num, actuator_state);
            #endif
        }
    #ifdef BUILD_LORA
        else if (interface_type_number == LORA_PWR) {
            lora.set_lora_enabled(actuator_state);
            queue_ack(pin_num, actuator_state);
        }
    #endif
#endif
    } else {
        #ifdef FLIGHT_COMPUTER
            iFlash.savedSensors = false;
            iFlash.setTimeout(millis());
        #endif

        add_sensors(pin_num, enable_rf, interface_type_number, sensors);
       // sensors_serial[pin_num] = interface_type_number | enable_rf << 6;

    }
    return true;
}

bool tx_motenet(SensorMap& sensors, ActuatorMap& actuators) {
    /**
     * @note sensor is a std::pair in the form of [KEY, OBJECT]
    */
   std::vector<mote_packet_t> packets_to_send;

   // Send a heartbeat
    if (remoteIP) {
        mote_packet_t packet = write_to_mote_packet(99, 0);
        packets_to_send.push_back(packet);
    }

    
    if (sensors.size() > 0 && remoteIP) {
        for (auto &sensor : sensors) {
            mote_packet_t packet = write_to_mote_packet(sensor.first, (int32_t) sensor.second->get_data());
            packets_to_send.push_back(packet);
        }
    }

    // send acks
    for (mote_packet_t ack : ack_q) {
        packets_to_send.push_back(ack);
        commands_acked++;
    }
    ack_q.clear();

    Udp.beginPacket(remoteIP, 8888);
    Udp.write((uint8_t*)packets_to_send.data(), sizeof(mote_packet_t) * packets_to_send.size());
    Udp.endPacket();

    return true;
}

/**
 * Update the network.
 * @param sensors The map of sensors to update.
 * @param actuators The map of actuators to update.
 * @return Whether the network was sucessfully updated.
*/

bool rx_motenet(SensorMap& sensors, ActuatorMap& actuators) {
    char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

    int packet_len = Udp.parsePacket();
    int packet_count = 0;
    const int max_packets_per_loop = 64;

    while (packet_len && packet_count < max_packets_per_loop) {
    
        IPAddress tmpIP = Udp.remoteIP();
        if (tmpIP) {
            remoteIP = tmpIP;
        }

        watchdog.recv_heartbeat();
        
        Udp.read(packetBuffer, packet_len);
        parse_mote_packet(packetBuffer, sensors, actuators);

        packet_len = Udp.parsePacket();
    }

    //Ethernet.maintain();
    
    return true;
}