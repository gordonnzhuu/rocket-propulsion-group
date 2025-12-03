#pragma once

#include "MinosData.h"
#include <SPI.h>
#include "FreeStack.h"
#include "BufferedPrint.h"

#ifdef FLIGHT_COMPUTER
    #include "Flash.h"
    #include "EthernetENC.h"
    #include "SdFat.h"
    #include "StateMachine.h"
#else
    #include <NativeEthernet.h>  
    #include "SD.h" 
#endif

#ifdef FLIGHT_COMPUTER
    extern SPIClass sd_lora_spi;
    #define SD1_CONFIG SdSpiConfig(PNUM_NOT_DEFINED, SHARED_SPI, SD_SCK_MHZ(1), &sd_lora_spi)
    #define SD1_CS PA10
#else
    /**
     * Use built-in SD card.
     * When writing to the CS pin, make it go into the void
     * "If anyone asks, it's not okay to write code like this"
     *      -- TE
    */
    #define SD1_CONFIG BUILTIN_SDCARD
    #define SD1_CS CORE_NUM_DIGITAL
#endif

class SDcard : public MinosData
{
    public:
        bool init();
        bool writeConfig(SensorMap& SensorMap, ActuatorMap& actuators);
        bool getConfig(SensorMap& SensorMap, ActuatorMap& actuators, AD7193 Adc0, AD7193 Adc1);
        bool writeData(uint8_t dataBuffer[], uint16_t dataLength);

        bool writeDataAll(SensorMap& SensorMap, ActuatorMap& actuators);
        bool writeActuators(ActuatorMap& actuators);

        bool getIPconfig(IPAddress *ip_addr);

        // change return type to int from uint_8t
        int getMoteNum();


        #ifdef FLIGHT_COMPUTER
            bool copyFlash(Flash& flash);
            bool writeState(flight_fsm_t new_state);
            flight_fsm_e readFlightState();
        #endif        

        #ifdef HITL
            SdFile hitlFile;
            char getNextHitlChar();
            bool checkHitlAvailible();
        #endif
    private:
        
        BufferedPrint<SdFile, 64> bp;
        BufferedPrint<SdFile, 64> bp2;
        uint32_t data_l = 100;
        char dataFilename [20] = "data1.csv";
        char actFilename [20] = "actuatorsNULL.csv";
        uint32_t position = 0;
        uint32_t dataWrites = 0;
        uint32_t actWrites = 0;

        uint32_t timeout = 0;

        SdFile dataFile;
        SdFile actuatorFile;
        SdFile flashFile;
        SdFile myFile;
        SdFile configFile;
        SdFile ipFile;
        SdFile actuatorFileOld;

        int file_num = 0;

         #ifdef FLIGHT_COMPUTER
            bool fwriteState = false;
            char state_change[20];
            SdFile stateFile;
         #endif

        AcuatorEvents act_events;
        
};