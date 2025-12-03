#include "BMP388.h"

#define SEALEVELPRESSURE_HPA 1013.25 //Get FAR pressure

MinosSensor m;
Adafruit_BMP3XX bmp;

 

bool BMP388::init(TwoWire i2c)
{
    TwoWire test = two;
    bool init_check = bmp.begin_I2C(BMP388::addr, &test);

    // Filters based  datasheet, but subject to change
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    return init_check;
}

bool BMP388::update()
{
    bool check = bmp.performReading();
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    return check;
}

uint32_t BMP388::get_data()
{
    return (uint32_t) altitude; 
}