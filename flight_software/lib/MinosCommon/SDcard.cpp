#include "SDcard.h"

#ifdef FLIGHT_COMPUTER
    SdFat SD;
#endif
// SdFatEX sd1;

int findGreatestFileNumber() {
    File root = SD.open("/");
    int maxNumber = 1;

    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            // No more files
            break;
        }

        if (!entry.isDirectory()) {
            char filename[32];
            #ifdef FLIGHT_COMPUTER
                entry.getName(filename, sizeof(filename));
            #else
                const char * file_name = entry.name();
                strcpy(filename, file_name);
            #endif
            if (strncmp(filename, "data", 4) == 0 && strcmp(filename + strlen(filename) - 4, ".csv") == 0) {
                int number;
                if (sscanf(filename + 4, "%d", &number) == 1) {
                    if (number > maxNumber) {
                        maxNumber = number;
                    }
                }
            }
        }
        entry.close();
    }
    root.close();

    return maxNumber;
}

bool SDcard::init()
{
    pinMode(SD1_CS, OUTPUT);
    digitalWrite(SD1_CS, HIGH);
    delay(1);
    digitalWrite(SD1_CS, LOW);

    volatile bool init_check = SD.begin(SD1_CONFIG); 

    for (int i = 0; i < 10; i++){
        if(init_check){
            break;
        }
        delay(100);
        init_check = SD.begin(SD1_CONFIG);
    }
    file_num = findGreatestFileNumber() + 1;
    bool file_exists = true;


    sprintf(dataFilename,"data%d.csv",file_num);
    if(!dataFile.open(dataFilename,  O_CREAT | O_RDWR)){
        init_check = false;
        return init_check;
    }else{
        init_check = dataFile.close();
        if(init_check){
            sprintf(actFilename,"actuators%d.csv",file_num);
            init_check = actuatorFile.open(actFilename,  O_CREAT | O_RDWR);
            if(init_check){
                init_check = actuatorFile.close();
            }
        }     
    }    


    #ifdef HITL
       init_check = hitlFile.open("hitl_data/data.csv", FILE_READ);
    #endif
    digitalWrite(SD1_CS, HIGH);


    return init_check;

}

/**
 * @param[out] ip_addr pointer to ip address to set.
*/

bool SDcard::getIPconfig(IPAddress* ip_addr) {
    digitalWrite(SD1_CS, LOW);
    bool ret = false;

    if (ipFile.open("ipConfig.txt")) {
        int id = ipFile.read() - '0' + 100;
        
        //This is creating a new IPAddress in memory at the address ip_addr
        //Fuck you, Bjarne Stroustrup
        new (ip_addr) IPAddress(192, 168, 1, id);

        ret = true;
        ipFile.close();
    }

    digitalWrite(SD1_CS, HIGH);
    return ret;
}

// change return type to int from uint_8t
int SDcard::getMoteNum() {
    digitalWrite(SD1_CS, LOW);
    int id = -1;

    if (ipFile.open("ipConfig.txt")) {
        id = ipFile.read() - '0';
        ipFile.close();
    }

    digitalWrite(SD1_CS, HIGH);
    return id;
}


bool SDcard::getConfig(SensorMap& sensors, ActuatorMap& actuators, AD7193 Adc0, AD7193 Adc1)
{
    
    // if(configFile.open("config.txt", FILE_READ) && configFile.available())
    // {
    //     // read until end of file
    //     while(configFile.available()){
    //         char line[CONFIG_LENGTH] = {0};
    //         for (int i = 0; i < CONFIG_LENGTH; i++){
    //             line[i] = configFile.read();
    //             if(line[i] == '\n'){
    //                 break;
    //             }
    //         }
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
    //             uint32_t pin_num = atoi(strtok(NULL,",")); 

    //             BNO * bno = new BNO();
    //             bno->set_config(config);

    //             sensors[pin_num] = bno;
    //         }
    //         if(strcmp("bmp",key)==0){
    //             float seaLevelP = atof(strtok(NULL,",")); //sea level pressure at location (FAR probably)
    //             uint32_t pin_num = atoi(strtok(NULL,",")); 

    //             BMP388 * bmp = new BMP388();
    //             bmp->set_seaLevel(seaLevelP);

    //             sensors[pin_num] = bmp;
    //         }
    //         if(strcmp("act",key)==0){
    //             uint32_t pin_num = atoi(strtok(NULL,",")); 

    //             MinosActuator * act = new MinosActuator;
    //             actuators[pin_num] = act;
    //         }
    //     }
    //     configFile.close();
    // return true;
    // }

    return false;   
}

bool SDcard::writeConfig(SensorMap& sensors, ActuatorMap& actuators){
    // configFile = SD.open("config.txt", FILE_WRITE);
    // if(configFile){
    //     for (auto &sensor : sensors) {
    //         if (AdcChannel *channel = dynamic_cast<AdcChannel *> (sensor.second)){
    //             char buf[8];
    //             sprintf(buf, "adc,%d\n", sensor.first); // this seems to work
    //             configFile.write(buf);
    //         }
    //         if (BNO *bno = dynamic_cast<BNO *> (sensor.second)){
    //             uint8_t config[22];
    //             char buf[30];
    //             if(bno->get_config(config))
    //                 configFile.write(config,22);
    //             sprintf(buf, "bno,%s,%d\n", config, sensor.first);
    //             configFile.write(buf);

    //         }
    //         if (BMP388 *bmp = dynamic_cast<BMP388 *> (sensor.second)){
    //             float data = bmp->get_seaLevel();
    //             char buf[6] = {0};
    //             char buf2[13] = {0};
    //             dtostrf(data,4,2,buf);
    //             sprintf(buf2,"bmp,%s,%d\n",buf,sensor.first);
    //             configFile.write(buf2);
    //         }
    //     }
    //     for(auto &actuator : actuators){
    //             char buf[9];
    //             sprintf(buf, "act,%d\n", actuator.first); // this seems to work
    //             configFile.write(buf);
    //     }
    // configFile.close();
    // return true;
    // }
    return false;
}

bool SDcard::writeDataAll(SensorMap& sensors, ActuatorMap& actuators){
    digitalWrite(SD1_CS, LOW);
    delayMicroseconds(1);

    if(!dataFile.isOpen()){
        if(!dataFile.open(dataFilename,  O_RDWR | O_APPEND)){
            digitalWrite(SD1_CS, HIGH);
            return false;
        }
    }

    uint32_t dt = millis();

    bp.begin(&dataFile);
    //dataFile.seekCur(position);
    data_l = 256;
    char buf[data_l] = {0};

    sprintf(buf, "%lu,", millis());
    for(auto &sensor : sensors)
    {
        sprintf(buf + strlen(buf),"%li,",sensor.second->get_data());
    }

    sprintf(buf + strlen(buf),"\n");
    data_l = strlen(buf);
    bp.write(buf, data_l);

    //dataFile.write(buf, data_l);
    position = dataFile.curPosition();
    dataWrites = dataWrites + 1;
    volatile bool synced;
    bp.sync(); // This does not actually save the file to the SD card

    if (dataWrites == 100) {
        dataWrites = 0;
        synced = dataFile.sync();
    }

    writeActuators(actuators);
    
    volatile uint32_t dt2 = millis() - dt;

    if(dt2 > 100){
        
       // init();
        digitalWrite(SD1_CS, HIGH);
        return false;
    }

    digitalWrite(SD1_CS, HIGH);
    return true;
}

bool SDcard::writeActuators(ActuatorMap& actuators){
    digitalWrite(SD1_CS, LOW);
    delayMicroseconds(1);
    
    if(!actuatorFile.isOpen()){
        if(!actuatorFile.open(actFilename,  O_RDWR | O_APPEND)){
            digitalWrite(SD1_CS, HIGH);
            return false;
        }
    }   
    bp2.begin(&actuatorFile);
    //dataFile.seekCur(position);
    data_l = 100;
    char buf[data_l] = {0};  
    volatile uint32_t num = act_events.size();
    volatile uint32_t len = actuators.size();
    //Actuators are hardware so the actuator map never changes
    //Populate act_events with the initial state of the actuators
    if(act_events.size() == 0){
        for(auto &actuator : actuators)
        {
            uint32_t id = actuator.first;
            act_events[id] = actuator.second->get_event();
            num = act_events.size();
            sprintf(buf + strlen(buf),"%d,%d,%d\n",id,act_events[id].time,act_events[id].state);
        }
    //Update act_events with the new events by checking time                
    }else{
        for(auto &actuator : actuators)
        {
            if(actuator.second->get_event().time != act_events[actuator.first].time){
                act_events[actuator.first] = actuator.second->get_event();
                sprintf(buf + strlen(buf),"%d,%d,%d\n",actuator.first,act_events[actuator.first].time,act_events[actuator.first].state);
            }   
        }
        num = act_events.size();
    }
    #ifdef FLIGHT_COMPUTER
        if(fwriteState){
            fwriteState = false;
            strcat(buf,state_change);
        }
    #endif

    data_l = strlen(buf);

    if(data_l == 0){
        digitalWrite(SD1_CS, HIGH);
        return false;
    }
    
    bp2.write(buf, data_l);

    volatile bool synced = bp2.sync();
    actuatorFile.sync();
    digitalWrite(SD1_CS, HIGH);
    return true;
}  

#ifdef FLIGHT_COMPUTER

bool SDcard::writeState(flight_fsm_t new_state){

    char buf[20] = {0};
    switch (new_state.state)
    {
    case IDLE:
        sprintf(buf, "STATE IDLE,%d\n", new_state.init_time);
        break;
    case LAUNCH:
        sprintf(buf, "STATE LAUNCH,%d\n", new_state.init_time);
        break;
    case COAST:
        sprintf(buf, "STATE COAST,%d\n", new_state.init_time);
        break;
    case DROGUE:
        sprintf(buf, "STATE DROGUE,%d\n", new_state.init_time);
        break;
    case MAINS:
        sprintf(buf, "STATE MAINS,%d\n", new_state.init_time);
        break;
    case ABORT:
        sprintf(buf, "STATE ABORT,%d\n", new_state.init_time);
        break;
    default:
        break;
    }
    fwriteState = true;
    strcpy(state_change,buf);
    // data_l = strlen(buf);
    // bp3.write(buf, data_l);
    // volatile bool synced = bp3.sync();
    // stateFile.sync();
    return true;
}

flight_fsm_e SDcard::readFlightState()
{
    digitalWrite(SD1_CS, LOW);
    
    flight_fsm_e flightState = IDLE;
    
    char filename[20] = {0};
    char lastStateLine[30] = {0};
    // Openining previous act file to find state
    sprintf(filename, "actuators%d.csv", file_num-1);
    if (actuatorFileOld.open(filename)) {
        while(actuatorFileOld.available()){
            char line[30] = {0};
            for (int i = 0; i < 30; i++){
                line[i] = actuatorFileOld.read();
                if(line[i] == '\n'){
                    break;
                }
            }
            if (strstr(line, "STATE") != NULL) {
                strcpy(lastStateLine, line);
            }
        }
        actuatorFileOld.close();

    }
    if (strlen(lastStateLine) > 0) {
        if (strstr(lastStateLine, "STATE IDLE") != NULL) {
            flightState = IDLE;
        } else if (strstr(lastStateLine, "STATE LAUNCH") != NULL) {
            flightState = LAUNCH;
        } else if (strstr(lastStateLine, "STATE COAST") != NULL) {
            flightState = COAST;
        } else if (strstr(lastStateLine, "STATE DROGUE") != NULL) {
            flightState = DROGUE;
        } else if (strstr(lastStateLine, "STATE MAINS") != NULL) {
            flightState = MAINS;
        } else if (strstr(lastStateLine, "STATE ABORT") != NULL) {
            flightState = ABORT;
        }
    }

    digitalWrite(SD1_CS, HIGH);
    return flightState;
}


bool SDcard::copyFlash(Flash& flash)
{
    digitalWrite(SD1_CS, LOW);

    if(flashFile.open("Flash.csv",  O_CREAT | O_RDWR)){
        bp.begin(&flashFile);

        bool eof = false;
        while (!eof){
            digitalWrite(SD1_CS, HIGH);
            data_l = flash.getDataLength();
            char buffer[100] = {0};
            flash.readDataLine(buffer);
            digitalWrite(SD1_CS, LOW);
            bp.write(buffer, 100);
            bp.sync();
            if(strstr(buffer,"EOF") != NULL){
                eof = true;
            }
        }
        flashFile.close();
        digitalWrite(SD1_CS, HIGH);

        return true;
        
    }
    digitalWrite(SD1_CS, HIGH);
    return false;
}
#endif


bool SDcard::writeData(uint8_t dataBuffer[], uint16_t dataLength)
{
    digitalWrite(SD1_CS, LOW);

    myFile.open("test.txt", FILE_WRITE);
    if(myFile)
    {
        myFile.write(dataBuffer, dataLength);
       // myFile.write((char *) dataBuffer);
        myFile.close();
    }
    else
    {
        digitalWrite(SD1_CS, HIGH);
        return false;
    }
    digitalWrite(SD1_CS, LOW);

    return true;
}

#ifdef HITL
char SDcard::getNextHitlChar() {
    digitalWrite(SD1_CS, LOW);
    return hitlFile.read();
}

bool SDcard::checkHitlAvailible() {
    digitalWrite(SD1_CS, LOW);
    return ((hitlFile.available() > 0) ? false : true);
}
#endif