#include <stdlib.h>
#include <vector>
#include <Arduino.h>
#include <SPI.h>
#include <NativeEthernet.h>    // Must use NativeEthernet libraries with Teensy
#include <NativeEthernetUdp.h> // Must use NativeEthernet libraries with Teensy
#include <RadioLib.h>          // LoRa Lib
#include "minos_util.h"
#include "LoraMap.h"

#define SX1262_SS 10
#define SX1262_DIO1 28
#define SX1262_DIO3 29
#define SX1262_NRST 6
#define SX1262_BUSY 8

#define freq 915.5           // Frequency
#define bw 125               // Bandwidth
#define sf 7                 // Spreading factor
#define cr 5                 // Coding rate denominator
#define syncWord 0x12        // Sync word
#define power 22             // Power in dbm
#define preambleLength 8     // Preamble length
#define tcxoVoltage 1.6      // TCXO reference voltage applied to DIO2 (in Volts)
#define useRegulatorLDO true // use LDO voltage regulator
#define ocp_limit 140        // set current limit to max

volatile bool receivedFlag = false;
int num_packets_recv = 0;
SX1262 radio = new Module(SX1262_SS, SX1262_DIO1, SX1262_NRST, SX1262_BUSY);

byte mac[6] = {0};
const int mote_num = 4;
EthernetUDP Udp;
IPAddress remoteIP;

struct SingleSensorConfig
{
  uint8_t pin_num;
  uint8_t interface_type_number;
  rf_config_t rf_config;
};
std::map<int, SingleSensorConfig> sensors;


void teensyMAC(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void init_ethernet() {
    Serial.println("Initializing Ethernet.");

    // Set up network
    teensyMAC(mac);
    IPAddress ip(192, 168, 1, 104);

    
    Ethernet.begin(mac, ip);
    Serial.println("Began Ethernet!");

    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Serial.println("No hardware!");
      while (true)
      {
        delay(10); // do nothing, no point running without Ethernet hardware
      }
    }

    while (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.print("Ethernet cable is not connected.");
      delay(1000);
    }

    // Starting UDP
    Serial.println("Starting UDP!");
    Udp.begin(8888);
    Serial.println(Ethernet.localIP());
}

void setFlag(void)
{
  receivedFlag = true; // we got a packet, set the flag
}

void init_lora() {
  // begin radio
  int state = radio.begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO);
  // int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("radio successfully started"));
  }
  else
  {
    Serial.print(F("radio startup failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio1Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1268] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.print(F("radio successfully started listening!"));
    Serial.println(state);
  }
  else
  {
    Serial.print(F("radio failed to begin listening, code "));
    Serial.println(state);
    while (true)
      ;
  }
}

mote_packet_t write_to_mote_packet(uint8_t pin_num, int32_t data) {
    mote_packet_t packet;
    packet.pin_num = pin_num;
    packet.data_bytes[0] = data & 0x000000ff;
    packet.data_bytes[1] = (data & 0x0000ff00) >> 8;
    packet.data_bytes[2] = (data & 0x00ff0000) >> 16;
    packet.data_bytes[3] = (data & 0xff000000) >> 24;

    return packet;
}

void parse_mote_packet(char* data) {
  uint8_t pin_num = data[0];
  uint8_t actuator_write_command = (data[1] & 0b10000000) == 0b10000000;
  // actuator_state and enable_rf are overloaded, since enable_rf is meaningless for actuators
  uint8_t actuator_state = (data[1] & 0b01000000) == 0b01000000;
  uint8_t enable_rf = (data[1] & 0b01000000) == 0b01000000;
  uint8_t interface_type_number = (data[1] & 0b00111111);

  if (actuator_state || actuator_write_command)
  {
    return;
  }

  if (interface_type_number == SENSOR_CLEAR_CODE) {
    Serial.println("Clearing sensors");
    sensors.clear();
    return;
  }

  SingleSensorConfig new_sensor;
  new_sensor.pin_num = pin_num;
  new_sensor.interface_type_number = interface_type_number;
  new_sensor.rf_config = {
    true, //rf_enable
    loraMap[interface_type_number].bits,
    loraMap[interface_type_number].scale
  };
  sensors[pin_num] = new_sensor;
}

uint32_t unpack_bits(uint8_t buf[], uint8_t bits, uint32_t start_bit) {
  uint32_t data = 0;

  for (uint8_t i = 0; i < bits; i++) {
    uint8_t buf_bit = (start_bit + i) % 8;
    uint8_t buf_byte = (start_bit + i) / 8;

    if ((buf[buf_byte] & (1 << buf_bit)))
      data |= (1 << i);
  }

  return data;
}

void parse_lora_packet(std::vector<mote_packet_t> &packets_to_send) {
    // Get how many bytes we read
    uint8_t num_bytes = radio.getPacketLength();
    if (num_bytes == 0) {
      Serial.println("Got packet of length zero!");
      return;
    }
    uint8_t buf[num_bytes];
    int state = radio.readData(buf, num_bytes);
    int offset = 0;

    if (state == RADIOLIB_ERR_NONE) // == 0
    {
      Serial.printf("Packet num: %d\n", num_packets_recv);

      Serial.print("Callsign: ");
      for (;offset < 6; offset++) Serial.print((char)buf[offset]);
      Serial.print('\n');

      uint16_t delta_time;
      memcpy(&delta_time, buf + offset, sizeof(delta_time));
      offset+=2;
      Serial.printf("Delta Time: %lu\n", delta_time);

      Serial.print("Processed Data: ");

      uint32_t current_bit = offset * 8;
      for (auto &sensor : sensors) {
        bool sig = unpack_bits(buf, 1, current_bit);
        uint32_t raw_value = unpack_bits(buf, sensor.second.rf_config.num_bits - 1, current_bit + 1);
        int32_t signed_value = raw_value * (sig ? -1 : 1);

        int32_t scaled_value = (int32_t)(signed_value * sensor.second.rf_config.scale);
        Serial.printf("%d ", scaled_value);

        current_bit += sensor.second.rf_config.num_bits;
        packets_to_send.push_back(
          write_to_mote_packet(sensor.first, scaled_value)
        );
      }
      Serial.print('\n');

      offset = 8;
      Serial.print("Raw Data: 0x");
      for (;offset < num_bytes; offset++) Serial.printf("%02X", buf[offset]);
      Serial.print('\n');
    }
    else // some error occurred
    {
      //write_to_telemetry_array(telemetry_to_send, state, 0); // write error message
      Serial.print("Error: ");
      Serial.println(state);
    }
    num_packets_recv++;
}

void setup() {
  // Set up Serial
  Serial.begin(9600);
  
  Serial.printf("MoTE Number: %d\n", mote_num);

  // Begin network
  init_ethernet();

  // Begin radio
  init_lora();
}

void loop() {
  // Recieve ethernet packet
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
  int packetSize = Udp.parsePacket();

  // Ethernet RX
  while (packetSize) {
    IPAddress tmpIP = Udp.remoteIP();
    if (tmpIP) {
        remoteIP = tmpIP;
    }

    Udp.read(packetBuffer, packetSize);
    parse_mote_packet(packetBuffer);
    packetSize = Udp.parsePacket();
  }

    // Radio RX
    std::vector<mote_packet_t> packets_to_send;
    if (receivedFlag) {
      parse_lora_packet(packets_to_send);
      receivedFlag = false;
    }

  // Ethernet TX
  // Heartbeat
  if (remoteIP) {
    mote_packet_t packet = write_to_mote_packet(99, 0);
    packets_to_send.push_back(packet);
  }

  if (remoteIP)
  {
    Udp.beginPacket(Udp.remoteIP(), 8888);
    Udp.write((uint8_t*)packets_to_send.data(), sizeof(mote_packet_t) * packets_to_send.size());
    Udp.endPacket();
  }
}


// byte *mac;
// std::vector<char> telemetry_to_send;
// EthernetUDP Udp;

// struct SingleSensorConfig
// {
//   uint8_t pin_num;
//   uint8_t interface_type_number;
//   uint8_t bit_length;
// };

// std::vector<SingleSensorConfig> all_sensors;

// // Buffers for receiving and sending data
// char packetBuffer[64];

// // Return the correct bit length for a sensor given the interface_type_number
// int lookupBitLength(int interface_type_number) {
//     // Look up the bit length for the given interface_type_number
//     auto it = loraMap.find(interface_type_number);
//     if (it != bitLengthMap.end()) {
//         return it->second;
//     }

//     // Default bit length if not found
//     return 0;
// }

// int32_t unpack_bits(uint8_t* buf, uint8_t bits, uint8_t start_bit) {
//   int32_t data = 0;

//   for (uint8_t i = 0; i < bits; i++) {
//     uint8_t buf_bit = (start_bit + i) % 8;
//     uint8_t buf_byte = (start_bit + i) / 8;

//     if ((buf[buf_byte] & (1 << buf_bit)))
//       data |= (1 << i);
//   }

//   return data;
// }

// void parsePacket(char *data)
// {
//   uint8_t pin_num = data[0];
//   uint8_t config_byte = data[1];
//   bool actuator_write_command = (config_byte & 0b10000000) == 0b10000000;
//   bool actuator_state = (config_byte & 0b01000000) == 0b01000000;
//   uint8_t interface_type_number = (config_byte & 0b00111111);

//   // This shouldn't happen unless someone sends an actuator config to this MoTE
//   if (actuator_state || actuator_write_command)
//   {
//     return;
//   }

//   if (interface_type_number == SENSOR_CLEAR_CODE) {
//     Serial.println("Clearing sensors");
//     all_sensors.clear();
//     return;
//   }

//   // if a sensor config command, save the sensor config
//   Serial.println("Received a sensor config command");
//   SingleSensorConfig new_sensor;
//   new_sensor.pin_num = pin_num;
//   new_sensor.interface_type_number = interface_type_number;
//   new_sensor.bit_length = lookupBitLength(interface_type_number);

//   all_sensors.push_back(new_sensor);

//   // Sort the vector based on pin_num in ascending order
//   std::sort(all_sensors.begin(), all_sensors.end(), [](const SingleSensorConfig &a, const SingleSensorConfig &b)
//             { return a.pin_num < b.pin_num; });
// }

// void write_to_telemetry_array(std::vector<char> &telemetry_to_send, int sensor_reading, int pin_num)
// {
//   // Treat the sensor reading as a pointer to its bytes to avoid issues with signedness
//   char *bytes = reinterpret_cast<char *>(&sensor_reading);

//   // Push the pin number to the telemetry array
//   telemetry_to_send.push_back(static_cast<uint8_t>(pin_num));

//   // Push each byte of the sensor reading to the telemetry array
//   for (uint i = 0; i < sizeof(sensor_reading); ++i)
//   {
//     telemetry_to_send.push_back(bytes[i]);
//   }
// }

// SX1262 radio = new Module(SX1262_SS, SX1262_DIO1, SX1262_NRST, SX1262_BUSY);
// void setFlag();

// // flag to indicate that a packet was received
// volatile bool receivedFlag = false;

// // this function is called when a complete packet
// // is received by the module
// // IMPORTANT: this function MUST be 'void' type
// // and MUST NOT have any arguments!
// void setFlag(void)
// {
//   receivedFlag = true; // we got a packet, set the flag
// }

// void setup()
// {
//   Serial.begin(9600);
//   Serial.print("Mote number: ");
//   Serial.println('4');

//   byte mac1[] = {0x36, 0x47, 0xA4, 0x6B, 0x46, 0x13}; // mote1 mac. might need to randomize
//   mac = &mac1[0];
//   IPAddress ip(192, 168, 1, 104);

//   Serial.println("about to begin ethernet");
//   Ethernet.begin(mac, ip);
//   Serial.println("Began Ethernet!");
//   if (Ethernet.hardwareStatus() == EthernetNoHardware)
//   {
//     Serial.print("Ethernet shield was not found. Cannot run without hardware.\n");
//     while (true)
//     {
//       delay(10); // do nothing, no point running without Ethernet hardware
//     }
//   }

//   Serial.println("After ethernet!");
//   while (Ethernet.linkStatus() == LinkOFF)
//   {
//     Serial.print("Ethernet cable is not connected.");
//     delay(1000);
//   }

//   // Start UDP
//   Serial.println("Starting UDP!");
//   Udp.begin(8888);
//   Serial.println(Ethernet.localIP());

//   // begin radio
//   int state = radio.begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO);
//   // int state = radio.begin();

//   if (state == RADIOLIB_ERR_NONE)
//   {
//     Serial.println(F("radio successfully started"));
//   }
//   else
//   {
//     Serial.print(F("radio startup failed, code "));
//     Serial.println(state);
//     while (true)
//       ;
//   }

//   // set the function that will be called
//   // when new packet is received
//   radio.setDio1Action(setFlag);

//   // start listening for LoRa packets
//   Serial.print(F("[SX1268] Starting to listen ... "));
//   state = radio.startReceive();
//   if (state == RADIOLIB_ERR_NONE)
//   {
//     Serial.print(F("radio successfully started listening!"));
//     Serial.println(state);
//   }
//   else
//   {
//     Serial.print(F("radio failed to begin listening, code "));
//     Serial.println(state);
//     while (true)
//       ;
//   }

//   // if needed, 'listen' mode can be disabled by calling
//   // any of the following methods:
//   //
//   // radio.standby()
//   // radio.sleep()
//   // radio.transmit();
//   // radio.receive();
//   // radio.scanChannel();
// } // end of setup()

// int packets_recv = 0;

// void loop()
// {

//   int packetSize = Udp.parsePacket();
//   if (packetSize)
//   {
//     // read the packet into packetBufffer
//     Udp.read(packetBuffer, 64);
//     parsePacket(packetBuffer);
//   }

//   if (receivedFlag)
//   {
//     // reset flag
//     receivedFlag = false;

//     // Get how many bytes we read
//     uint8_t num_bytes = radio.getPacketLength();
//     if (num_bytes == 0) {
//       Serial.println("Got packet of length zero!");
//       return;
//     }
//     uint8_t buf[num_bytes];
//     int state = radio.readData(buf, num_bytes);
//     int offset = 0;

//     if (state == RADIOLIB_ERR_NONE) // == 0
//     {
//       Serial.printf("Packet num: %d\n", packets_recv);

//       Serial.print("Callsign: ");
//       for (;offset < 6; offset++) Serial.print((char)buf[offset]);
//       Serial.print('\n');

//       uint16_t delta_time;
//       memcpy(&delta_time, buf + offset, sizeof(delta_time));
//       offset+=2;
//       Serial.printf("Delta Time: %lu\n", delta_time);

//       Serial.print("Processed Data: ");

//       uint32_t current_bit = offset * 8;
//       for (const SingleSensorConfig &sensor : all_sensors) {
//         int32_t datum = unpack_bits(
//           buf, lookupBitLength(sensor.interface_type_number), current_bit
//         );

//         Serial.print(datum);
//         Serial.print(" ");

//         current_bit += lookupBitLength(sensor.interface_type_number);
//         write_to_telemetry_array(telemetry_to_send, datum, sensor.pin_num);
//       }
//       Serial.print('\n');

//       offset = 8;
//       Serial.print("Raw Data: 0x");
//       for (;offset < num_bytes; offset++) Serial.printf("%02X", buf[offset]);
//       Serial.print('\n');

//       Serial.printf("RSSI = %f\n", radio.getRSSI());
//     }
//     else // some error occurred
//     {
//       //write_to_telemetry_array(telemetry_to_send, state, 0); // write error message
//       Serial.print("Error: ");    radio.setCurrentLimit(ocp_limit); // Set OCP to 140mA to allow high-power TX

//       Serial.println(state);
//     }
//     packets_recv++;
//   } // end of if (receivedFlag)

//   // Udp.parsePacket();
//   if (telemetry_to_send.size() == 0)
//   {
//     write_to_telemetry_array(telemetry_to_send, random(0, 101), 99); // heartbeat or something ig
//   }

//   if (Udp.remoteIP())
//   {
//     Udp.beginPacket(Udp.remoteIP(), 8888);
//     Udp.write(telemetry_to_send.data(), telemetry_to_send.size());
//     Udp.endPacket();
//   }

//   telemetry_to_send.clear();
// } // end of loop()