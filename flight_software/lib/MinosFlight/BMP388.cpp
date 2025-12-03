#include "BMP388.h"
#ifdef HITL
    #include "HitlManager.h"    
#endif

#ifdef HITL
    extern HitlManager hitl;
#endif

volatile bool bmp_drdy = false;

void interrupt_handler_bmp388() {
    bmp_drdy = true;
}

bool BMP388::init()
{
    bmp388.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
    bmp388.enableInterrupt();                       // Enable the BMP388's interrupt (INT) pin
    bmp388.setPresOversampling(OVERSAMPLING_X2);
    bmp388.setTempOversampling(OVERSAMPLING_SKIP);
    attachInterrupt(PD2, interrupt_handler_bmp388, RISING);   // Set interrupt to call interruptHandler function on D2
    bmp388.setTimeStandby(TIME_STANDBY_10MS);     // Set the standby time to 1.3 seconds
    bmp388.setSeaLevelPressure(seaLevelPressure); // Set the sea level pressure to 1013.25 hPa
    bmp388.startNormalConversion();                 // Start BMP388 continuous conversion in NORMAL_MODE  


    #ifdef HITL
        while(!bmp_drdy){
            delay(100);
        }
            float pres = 1018.9;
            bmp388.getPressure(pres);
            bmp_drdy = false;
            bmp388.setSeaLevelPressure(pres);
            bmp388.startNormalConversion(); 
            
    #endif


    return true;
}

bool BMP388::getinitialized()
{
    return initialized;
}

void BMP388::set_ground_alt() {
    ground_alt_set = false;
}

bool BMP388::update()
{
    float temp, pres, alt;
    

    if (bmp_drdy)
    {  
        bmp388.getMeasurements(temp, pres, alt);

        if (!ground_alt_set) {
            ground_altitude = alt;
            ground_alt_set = true;
        }

        altitude = alt - ground_altitude;
        temperature = temp;
        pressure = pres;

        bmp_drdy = false;

        #ifdef HITL
            if(hitl.get_hitl_mode() == HitlManager::MODE_REAL_TIME)
                altitude = hitl.get_current_alt();
        #endif
    } 

    return true;
}

void BMP388::set_seaLevel(float seaPressure)
{
    seaLevelPressure = seaPressure;
}

float BMP388::get_seaLevel()
{
   return seaLevelPressure;
}

int32_t BMP388::get_data(channel_config_t channel)
{
    switch (channel)
  {
    case BMP_ALT:
        return (int32_t)(altitude*1000); 
    case BMP_TMP:
        return (int32_t) (temperature*1000);
    case BMP_P:
        return (int32_t) (pressure * 1000); // Multiply by 1000 to get dPa (decipascals). Rember to divide by 1000 to get hPa in GUI.
    default:
        return (int32_t) (pressure);
  }
}