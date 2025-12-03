// Lora.cpp
// Remus Harris
// December 20, 2024

#include "Lora.h"

volatile bool tx_ready = true; // Transmission state flag

bool Lora::init() {
    spi.begin();

    // Initialize SX1262 with custom parameters
    digitalWrite(SX1262_SS, LOW);
    int state = radio.begin(freq, bw, sf, cr, syncWord, lora_power, preambleLength, tcxoVoltage, useRegulatorLDO);
    radio.setCurrentLimit(ocp_limit); // Set OCP to 140mA to allow high-power TX
    volatile int16_t status;

    if (state == RADIOLIB_ERR_NONE) {
        lora_instance = this; // Link static instance
        radio.setDio1Action(on_transmit_done); // Set callback
        tx_ready = true;

        state = set_lora_enabled(false);
        status = radio.standby();
        digitalWrite(SX1262_SS, HIGH);
        return state;
    }

    digitalWrite(SX1262_SS, HIGH);
    return false;
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

void Lora::pack_bits(uint8_t buf[], uint32_t data, uint8_t bits, uint32_t start_bit) {
  for (uint8_t i = 0; i < bits; i++) {
    uint8_t buf_bit = (start_bit + i) % 8;
    uint8_t buf_byte = (start_bit + i) / 8;

    if ((data & (1 << i)))
      buf[buf_byte] |= (1 << buf_bit);
  }
}



bool Lora::send_data(SensorMap &sensors) {
    if (!lora_enabled) {
        return true; // lora is off
    }
    volatile int16_t status;

    // // If the lora has been "busy" for 1.5s, we done fucked up.
    // if (!tx_ready && millis() - last_send_time > lora_reset_timeout) {
    //     radio.reset();
    //     return false;
    // }

    if (tx_ready) {
        // Assert pin low.

        digitalWrite(SX1262_SS, LOW);
       status=  radio.finishTransmit();
        digitalWrite(SX1262_SS, HIGH);

        uint8_t buf[256] = {0}; // Length of LoRa buffer
        size_t offset = 0;

        // Start by prepending the callsign
        for (int i = 0; i < 6; i++) {
            buf[offset] = callsign[i];
            offset++;
        }

        // Append the delta time in 2 bytes
        uint16_t delta_time = millis() - last_send_time;
        last_send_time = millis();
        memcpy(buf + offset, &delta_time, sizeof(delta_time));
        offset += 2;

        uint32_t current_bit = offset * 8;

        for (auto &sensor : sensors) {
            rf_config_t rf_config = sensor.second->get_rf_config();
            volatile int pin_num = sensor.first;

            // If we don't want to RF this sensor, skip it.
            if (!rf_config.enable) {
                continue;
            }

            int32_t rf_data = (int32_t)(sensor.second->get_data() / rf_config.scale);

            uint8_t sig = rf_data < 0 ? 1 : 0;
            pack_bits(buf, sig, 1, current_bit);
            volatile bool tmp_sig = unpack_bits(buf, 1, current_bit);

            uint32_t val = (uint32_t)abs(rf_data);
            pack_bits(buf, val, rf_config.num_bits - 1, current_bit + 1);
            volatile uint32_t tmp_val = unpack_bits(buf, rf_config.num_bits - 1, current_bit + 1);
            
            volatile int32_t tmp = tmp_val * (tmp_sig ? -1 : 1);

            if (rf_data != tmp) {
                __NOP();
            }

            current_bit += rf_config.num_bits;

            // Divide + round up
            offset = (current_bit + 7) / 8;

            if (offset > sizeof(buf)) return false;
        }

        // Start the transmission
        tx_ready = false; // Mark as transmitting
        digitalWrite(SX1262_SS, LOW);
        int state = radio.startTransmit(buf, offset, 0x0);
        digitalWrite(SX1262_SS, HIGH);

        if (state != RADIOLIB_ERR_NONE) {
            tx_ready = true; // Error state, not transmitting.
            return false;
        }
    }
    return true;
}

bool Lora::get_tx_ready() {
    return tx_ready; // Check transmission state
}

// Called when DIO1 signals ready (no clue if that's 0 or 1 lol)
void Lora::on_transmit_done() {
    tx_ready = true;
}

bool Lora::set_lora_enabled(bool state) {
    lora_enabled = state;
    int16_t status;
    
    digitalWrite(SX1262_SS, LOW);
    if (lora_enabled) {
        //status = radio.standby();
        tx_ready = true;
    } else {
       // status = radio.sleep();
    }
    digitalWrite(SX1262_SS, HIGH);

    return status == RADIOLIB_ERR_NONE;
}

void Lora::set_lora_power(uint8_t new_power) {
    digitalWrite(SX1262_SS, LOW);
    radio.setOutputPower(new_power);
    digitalWrite(SX1262_SS, HIGH);
}