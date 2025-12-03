//#pragma once
#include "internalFlash.h"
#include "stm32f4xx_hal_flash.h"
#include "EEPROM.h"
#include "utility/stm32_eeprom.h"
#include "../../src/network.h"
#include "AD7193.h"
#include "AdcChannel.h"
#include "LoopTimer.h"
#include "Watchdog.h"
#include "BMP388.h"
#include "BMPChannel.h"
#include "BNO.h"
#include "BNOChannel.h"
#include "VoltMonitor.h"
#include "StateMachine.h"
#include "AlphaBetaFilter.h"
#include "GNCChannel.h"
#include "StateChannel.h"

#define EEPROM_START_ADDRESS 0x080FF004 // Writing on the last page of the flash memory
#define EEPROM_END_ADDRESS 0x080FFFFF

extern SensorSerialMap sensors_serial;
// Erase the sector 11 of the flash memory
bool internalFlash::init()
{
    uint32_t error;
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_11;
    erase.NbSectors = 1;
    erase.VoltageRange = VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &error);
    volatile uint32_t error2 = error;

    HAL_FLASH_Lock();
    return true;
}

bool parse_flash(uint16_t val, SensorMap& sensors)
{
uint8_t pin_num = (val & 0xFF00) >> 8;
uint8_t interface_type_number = val & 0x00FF;
uint8_t enable_rf = (interface_type_number & 0b01000000) == 0b01000000;
interface_type_number = val & 0x00FF;


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
                sensors[80]->init();
        }
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
        } else if (interface_type_number == SENSOR_CLEAR_CODE) {
                sensors.clear();
                sensors_serial.clear();
        }
    sensors_serial[pin_num] = interface_type_number;
return true;
}

bool internalFlash::saveSensors(){
    uint32_t address = EEPROM_START_ADDRESS;
    volatile uint64_t val;
    volatile HAL_StatusTypeDef status;

    // Erase the sector before each write
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_11;
    erase.NbSectors = 1;
    erase.VoltageRange = VOLTAGE_RANGE_3;
    status = HAL_FLASH_Unlock();
    
    val = *(uint32_t*)EEPROM_START_ADDRESS;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
    uint32_t error;
    status = HAL_FLASHEx_Erase(&erase, &error);
    volatile uint64_t error2 = error;

    val = *(uint32_t*)EEPROM_START_ADDRESS;

    // Reset flags in order to write    

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);


    bool padding = (sensors_serial.size() % 2) !=0;
    uint8_t length = sensors_serial.size() / 2 + uint8_t(padding);
    uint32_t sensor_vec32[length];
    uint16_t sensor_vec16[sensors_serial.size()+2*uint8_t(padding)];
    int i = 0;
    memset(sensor_vec16, 0, sizeof(sensor_vec16));
    
    for(auto &sensor : sensors_serial)
    {
       sensor_vec16[i] = sensor.first << 8 | sensor.second;
        i++;
    }
    memset(sensor_vec32, 0, sizeof(sensor_vec32));
    
    for(int index = 0; index < length; index++)
    {
        sensor_vec32[index] = sensor_vec16[2*index] << 16 | sensor_vec16[2*index+1];    
    }
    int index;
    for(index = 0; index < length; index++)
    {
        address += 4;
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t) sensor_vec32[index]);
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address+4, (uint64_t) 0xDEADBEEF);

    FLASH_FlushCaches();

    volatile uint32_t statusi = HAL_FLASH_GetError();
    val = *(uint32_t*)EEPROM_START_ADDRESS;
    val = *(uint32_t*)(address+4);
    status = HAL_FLASH_Lock();
    __NOP();
    savedSensors = true;
    return bool (status == HAL_OK);
}

bool internalFlash::setFlightState(flight_fsm_e flightState)
{
    uint32_t time = millis();
    uint32_t error;
    uint8_t state = (uint8_t) flightState;
    volatile HAL_StatusTypeDef status;


    // Erase the sector before each write
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_11;
    erase.NbSectors = 1;
    erase.VoltageRange = VOLTAGE_RANGE_3;
    status = HAL_FLASH_Unlock();
    
    volatile uint64_t val = *(uint32_t*)EEPROM_START_ADDRESS;
    // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
    // status = HAL_FLASHEx_Erase(&erase, &error);
    // volatile uint64_t error2 = error;

    val = *(uint32_t*)EEPROM_START_ADDRESS;

    // Reset flags in order to write    

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
    
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, EEPROM_START_ADDRESS, (uint64_t) state);

    FLASH_FlushCaches();

    volatile uint32_t statusi = HAL_FLASH_GetError();
    val = *(uint32_t*)EEPROM_START_ADDRESS;
    status = HAL_FLASH_Lock();
    volatile uint32_t dt = millis() - time;
    __NOP();
    return bool (status == HAL_OK);
}

uint8_t internalFlash::readFlightSensors(SensorMap& sensors, ActuatorMap& actuators)
{

    volatile HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    uint64_t val = 255;

    uint16_t length = 0;
    uint32_t address = EEPROM_START_ADDRESS + 4;
    while(true)
    {
        val = *(uint32_t*)(address + length*4); 
        if(val == 0xDEADBEEF || length > 100)
        {
            break;
        }
        uint8_t data[4];
        data[0] = (val & 0x0000FF00) >> 8;
        data[1] = val & 0x000000FF;
        data[2] = (val & 0xFF000000) >> 24;
        data[3] = (val & 0x00FF0000) >> 16;

       uint8_t enable_rf = (data[3] & 0b01000000) == 0b01000000;
       uint8_t interface_type_number = (data[3] & 0b00111111);

        add_sensors(data[2], enable_rf, interface_type_number, sensors);
        
        enable_rf = (data[1] & 0b01000000) == 0b01000000;
        interface_type_number = (data[1] & 0b00111111);

        add_sensors(data[0], enable_rf, interface_type_number, sensors);

        

        length++;
        
    }
    uint32_t error;
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_11;
    erase.NbSectors = 1;
    erase.VoltageRange = VOLTAGE_RANGE_3;
    
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
    status = HAL_FLASHEx_Erase(&erase, &error);


    HAL_FLASH_Lock();
    // Make sure it is a valid state incase of corruption
    volatile uint8_t num = sensors.size();

    return sensors.size();
}

