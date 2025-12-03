#include "Flash.h"

#define MemCs PC13 //chip select
#define MemWp PE1 //write protection
#define MemHold PE0 //hold

bool Flash::init()
{

    // SPI.setMOSI(PC12);
    // SPI.setMISO(PC11);
    // SPI.setSCLK(PC10);
     //SPI.begin(PC13);

    Flash::flash.begin(Flash::cs_pin,MemWp,MemHold,idbuffer);
    if(strcmp("BF 25 41",idbuffer) == 0) //TODO: Get real id
    {
        return true;
    }
    //MinosData::spi3.begin();
    return false;
}

bool Flash::write(uint8_t dataBuffer[], uint16_t dataLength)
{
    
    waddress = flash.writeArray(waddress, dataBuffer, dataLength) + 1;
    
    return true;
}
bool Flash::getConfig(SensorMap& sensors, ActuatorMap& actuators, AD7193 Adc0, AD7193 Adc1)
{
    // char line[CONFIG_LENGTH] = {0};
    // flash.readArray(0, (uint8_t*)line, 9); // reading the CONFIG:\n
    // raddress = 9;
    // if(strcmp(line,"CONFIG:\n") == 0)
    // {
    //      bool done = false;
    //     do
    //     {
    //         flash.readArray(raddress, (uint8_t*)line, CONFIG_LENGTH);
    //         raddress += CONFIG_LENGTH;
    //         char* key = strtok(line,",");
    //         char* value;
    //         if(strcmp(key,"adc")==0){
    //             value = strtok(NULL,",");
    //             uint32_t pin_num = atoi(value); 
                  
    //             AdcChannel * channel = new AdcChannel(
    //                 pin_num % 10, //channel
    //                 pin_num / 10 == 0 ? Adc0 : Adc1
    //             );
    //             sensors[pin_num] = channel;
    //         }
    //         if(strcmp("bno",key)==0){
    //             uint8_t* config = (uint8_t*) strtok(NULL,","); // 22 byte config should be there
    //             value = strtok(NULL,",");
    //             uint32_t pin_num = atoi(value);

    //             BNO * bno = new BNO();
    //             bno->set_config(config);

    //             sensors[pin_num] = bno;
    //         }
    //         if(strcmp("bmp",key)==0){
    //             float seaLevelP = atof(strtok(NULL,",")); //sea level pressure at location (FAR probably)
    //             value = strtok(NULL,",");
    //             uint32_t pin_num = atoi(value); 

    //             BMP388 * bmp = new BMP388();
    //             bmp->set_seaLevel(seaLevelP);

    //             sensors[pin_num] = bmp;
    //         }
    //         if(strcmp("act",key)==0){
    //             value = strtok(NULL,",");
    //             uint32_t pin_num = atoi(value);

    //             MinosActuator * act = new MinosActuator;
    //             actuators[pin_num] = act;
    //         }

    //         if(strstr(value,"\n") == NULL)
    //         {
    //             done = true;
    //         }
    
    //     }  while(!done); // read until no new end of lines
    //     return true;
    // }
      
    // return false;
}

bool Flash::writeConfig(SensorMap& sensors, ActuatorMap& actuators){
//     flash.sectorErase(0);
//    waddress = flash.writeArray(0, (uint8_t*)"CONFIG:\n", 9);
//         for (auto &sensor : sensors)   
//         {
//             char buf[CONFIG_LENGTH] = {0};

//             if (AdcChannel *channel = dynamic_cast<AdcChannel *> (sensor.second))
//             {
//                 sprintf(buf, "adc,%d\n", sensor.first);
//             }
//             if (BNO *bno = dynamic_cast<BNO *> (sensor.second))
//             {
//                 uint8_t config[22];
//                 bno->get_config(config);
//                 sprintf(buf, "bno,%s,%d\n", config, sensor.first);
//             }
//             if (BMP388 *bmp = dynamic_cast<BMP388 *> (sensor.second))
//             {
//                 char buf2[6] = {0};
//                 dtostrf(bmp->get_seaLevel(),4,2,buf2);
//                 sprintf(buf, "bmp,%s,%d\n", buf2, sensor.first);
//             }

//             waddress = flash.writeArray(waddress, (uint8_t*) buf, CONFIG_LENGTH);

//         }

//         for(auto &actuator : actuators)
//         {
//             char buf[CONFIG_LENGTH] = {0};
//             sprintf(buf, "act,%d\n", actuator.first);
//             waddress = flash.writeArray(waddress, (uint8_t*) buf, CONFIG_LENGTH);
//         }
    
    return true;
    }
bool Flash::writeDataAll(SensorMap& sensors, ActuatorMap& actuators)
{
    if (waddress < 512){
        waddress = 512;
        char buf[10] = {0};
        sprintf(buf + strlen(buf),"DATA:\n");
        flag_length = strlen(buf);
        waddress = flash.writeArray(waddress, (uint8_t*) buf, flag_length);
    }
   
    data_l = 100;
    char buf[data_l] = {0};

    sprintf(buf, "%lu,", millis());
    for(auto &sensor : sensors)
    {

        sprintf(buf + strlen(buf),"%d,%li,",sensor.first,sensor.second->get_data());
    }
    for(auto &actuator : actuators)
    {
        sprintf(buf + strlen(buf),"%d,%d,",actuator.first,actuator.second->get_state());
    }
    sprintf(buf + strlen(buf),"%lu,\n",millis());
    uint32_t dt = millis();
    waddress = flash.writeArray(waddress, (uint8_t*) buf, 100);
    volatile uint32_t t = millis() - dt;
    data_l = strlen(buf);
    char endbuf[10] = "\tEOF\n";

    flash.writeArray(waddress, (uint8_t*) endbuf, strlen(endbuf));
    t = millis() - dt;

    return true;
}

bool Flash::readDataLine(char* buf)
{
    while(!data_flag){
    if (raddress < 512){
        raddress = 512;
        char line[10] = {0};
        flash.readArray(raddress, (uint8_t*)line, flag_length);
        raddress += flag_length;
        if(strcmp(line,"DATA:\n") == 0){
            data_flag = true;
        }
    }else{
        break;
    }

    }
    //char buffer[100] = {0};
    flash.readArray(raddress, (uint8_t*)buf, 100);
    // strtok(buffer,"\t");
    // buf = strtok(NULL,"\n");
   // sprintf(buf + strlen(buf),"\n");
    raddress += 100;
    return true;
}

bool Flash::clearFlash()
{
    flash.totalErase();
    return true;
}

uint32_t Flash::getDataLength()
{
    return data_l;
}

uint8_t Flash::read(uint16_t addr)
{
    flash.readInit(addr);
    uint8_t result = flash.readNext();
    flash.readFinish();
    return result;
}