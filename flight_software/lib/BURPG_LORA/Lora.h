// Lora.h
// Remus Harris
// December 20, 2024

#pragma once

#include "SPI.h"
#include "RadioLib.h"
#include "minos_util.h"

#define SX1262_SS PC14 // SPI3_CS2
#define SX1262_DIO1 PA8 // I2C3_SCL
#define SX1262_NRST PC9 // I2C3_SDA
#define SX1262_BUSY PC8  // SPI3_CS3

#define freq 915.5           // Frequency
#define bw 125               // Bandwidth
#define sf 7                 // Spreading factor
#define cr 5                 // Coding rate denominator
#define syncWord 0x12        // Sync word
#define lora_power 16             // Power in dbm
#define preambleLength 8     // Preamble length
#define tcxoVoltage 1.6      // TCXO reference voltage applied to DIO2 (in Volts)
#define useRegulatorLDO true // use LDO voltage regulator
#define ocp_limit 140        // set current limit to max

// HAM callsign to include to make all this legal
// Using Tim Evdokimov's callsign, exp. August 2025
static const char callsign[7] = "KC1EFZ";

class Lora {
public:
    Lora(uint32_t cs_pin, uint32_t gpio1, uint32_t rst, uint32_t irq, SPIClass &loraSpi) :
        cs_pin(cs_pin), 
        gpio1(gpio1),
        rst(rst), 
        irq(irq),  
        spi(loraSpi),
        radio(new Module(cs_pin, gpio1, rst, irq, spi)) {}

    bool init();
    bool send_data(SensorMap &sensors);
    bool get_tx_ready(); // Check transmission state

    bool set_lora_enabled(bool state);
    void set_lora_power(uint8_t new_power);

private:
    uint32_t cs_pin;
    uint32_t gpio1;
    uint32_t rst;
    uint32_t irq;
    SPIClass &spi;
    SX1262 radio;
    uint32_t last_send_time = 0;
    bool lora_enabled = false;

    static void on_transmit_done();
    void pack_bits(uint8_t* buf, uint32_t data, uint8_t bits, uint32_t start_bit);

    const uint32_t lora_reset_timeout = 1500;
};

// Define a static pointer for callback linkage
static Lora *lora_instance = nullptr;