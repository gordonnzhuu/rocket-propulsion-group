#include "Arduino.h"
#include "SPI.h"

//Macros relevant to the Communications register... (the first byte transferred to the AD7193 in a transaction will always be written to the Comm register, so it has no unique address)
//Bits 0, 6, and 7 must always be empty (0). Bit 1 = r/w ; bits 2-4 = register address ; bit 5 = enable continuous read (has an effect for data reg reads only)
#define AD7193_REG_READ 0b01000000            //Signals a read operation
#define AD7193_REG_WRITE 0b00000000           //Signals a write operation
#define AD7193_STATUS_REG 0b00000000        //8-bit reg (r)
#define AD7193_MODE_REG 0b00001000          //24-bit reg (r/w)  
#define AD7193_CONFIG_REG 0b00010000        //24-bit reg (r/w)
#define AD7193_DATA_REG 0b00011000          //24-bit reg (r)
#define AD7193_ID_REG 0b00100000            //8-bit reg (r)
#define AD7193_GPOCON_REG 0b00101000        //8-bit reg (r/w)
#define AD7193_OFFSET_REG 0b00110000        //24-bit reg (r/w)
#define AD7193_FULLSCALE_REG 0b00111000     //24-bit reg (r/w)

#define AD7193_CHANNEL_BIT 8
#define AD7193_DIFFERENTIAL_BIT 18
#define AD7193_PGA_BIT 0
#define AD7183 POLARITY_BIT 3
#define AD7193_CHANNEL_SHIFT 0x00000100

#define VREF 5
#define AD7193_CODE_EXP_TERM 16777216   //This is 2^24

struct AD7193_driver_arg_t {
    uint8_t comm_bits;
    uint32_t write_payload;
};

struct AD7193_reading_t
{
    uint8_t channel;
    double result;
};

double AD7193_codeToVolts(uint32_t code, uint16_t currentGainSetting, bool unipolar){
    int gain = 1;

    switch(currentGainSetting){
        case 0x00: break;
        case 0x03: gain = 8; break;
        case 0x04: gain = 16; break;
        case 0x05: gain = 32; break;
        case 0x06: gain = 64; break;
        case 0x07: gain = 128; break;
    }

    //These formulas both appear on page 33 of the AD7193 data sheet!!!
    if(unipolar)        //Evaluates to 'true' if ADC is in unipolar mode! Following is the formula for the unipolar case...
        return ((double)(code * VREF)) / ((double)(AD7193_CODE_EXP_TERM));

    //If the above conditional statement fails, then the ADC is in bipolar mode! Following is the formula for the bipolar case...
    return ( ( (double)code / ((double)AD7193_CODE_EXP_TERM / 2) ) - 1 ) / ( (double) gain/ (double)VREF );
}


uint32_t AD7193_driver(AD7193_driver_arg_t args){
    //Basic init stuff -- figure out which SPI bus we're using, then get and set the right pins accordingly...
    // pinMode(cs_pin, OUTPUT);
    // digitalWrite(cs_pin, LOW);
    //ADC enters a low power mode after making a conversion, so we have to wait for it to fully power up before communicating with it to avoid problems...
    
    uint8_t reg_id = args.comm_bits & 0b00111000;
    if((args.comm_bits & AD7193_REG_READ) && reg_id == AD7193_DATA_REG){
        while(1){
            if(digitalRead(12) == LOW)
                break;
        }
    }

    //TODO WARNING this statement is not thread safe. Make it so when integrating with MINOS
    SPI.transfer(args.comm_bits);
    
    if(args.comm_bits & AD7193_REG_READ){
        //Reading data from one of the ADC's registers...
        //Readable registers in the AD7193 have one of two sizes: 8-bit, and 24-bit. The 8-bit registers are STATUS, ID, and GPOCON, with all others being 24-bit...
        if(reg_id == AD7193_STATUS_REG || reg_id == AD7193_ID_REG || reg_id == AD7193_GPOCON_REG)
            return SPI.transfer(0);
        else{
            //Read operations from the data register trigger a fresh conversion in the ADC, so we have to wait for the result to settle before reading...
            //The ADC signals that it's ready to output a reading by bringing its D_OUT pin low (without being triggered by the clock), so we wait for this to happen before proceeding...

            uint32_t sensor_return = 0;
            //Bits are delivered to us going from MSB to LSB...
            //Include logic here to account for whether or not the status register bits are being appended!!!!
            sensor_return |= SPI.transfer(0) << 24;
            sensor_return |= SPI.transfer(0) << 16;
            sensor_return |= SPI.transfer(0) << 8;
            sensor_return |= SPI.transfer(0);
            return sensor_return;
        }
        //End of READ if-block...
    }
    else{
        //Writing data to one of the ADC's registers...
        //Only one of the AD7193's writeable registers has 8-bit size -- the rest are 24-bit...
        if(reg_id == AD7193_GPOCON_REG)
            SPI.transfer((uint8_t)args.write_payload);
        else{
            //We transfer the bits going from MSB to LSB...
            SPI.transfer((uint8_t)(args.write_payload >> 16));
            SPI.transfer((uint8_t)(args.write_payload >> 8));
            SPI.transfer((uint8_t)args.write_payload);
        }
        return 0;
        //End of WRITE else-block
    }
    //Deselect device to avoid spurious writes...
    //digitalWrite(cs_pin, HIGH);
}

bool update() {
	AD7193_reading_t new_reading[2];
    AD7193_driver_arg_t driver_args;

	uint16_t gain_bits;
    switch(1){
        case 1:     gain_bits = 0x0;    break;
        case 8:     gain_bits = 0x03;   break;
        case 16:    gain_bits = 0x04;   break;
        case 32:    gain_bits = 0x05;   break;
        case 64:    gain_bits = 0x06;   break;
        case 128:   gain_bits = 0x07;   break;
        default:    gain_bits = 0x0;
    }
    uint32_t config_write_payload = 0x0 | (!false << AD7193_DIFFERENTIAL_BIT) | ((1 << 0) << 8) | (gain_bits);
    uint32_t config_write_payload_differential = 0x04FF00; // all channels
    uint32_t config_write_payload_pt = 0x041F00; // for pressure transducer
    uint32_t config_write_payload_tc = 0x000107; // for thermocouple
    uint32_t mode_reg_payload_calib = 0x080060 | (1 << 20) | (0b001 << 21); // calibrates adc, should be done on startup
    uint32_t mode_reg_payload_cont = 0x180001; // continuous read mode
    uint32_t mode_reg_payload_idle = 0x580001; // idle
    uint32_t mode_reg_payload_pwrdwn = 0x780001; // pwrdwn

    pinMode(10, OUTPUT);
  
    digitalWrite(10, LOW);
    // for (uint8_t i = 0; i < 6; i++)
    // {
    //    SPI.transfer(0xFF);
    // }
    
    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_CONFIG_REG;
    driver_args.write_payload = config_write_payload;
    AD7193_driver(driver_args);

    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_MODE_REG;
    driver_args.write_payload = mode_reg_payload_calib;
    AD7193_driver(driver_args);

    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_MODE_REG;
    driver_args.write_payload = mode_reg_payload_cont;
    AD7193_driver(driver_args); 

    driver_args.comm_bits = AD7193_REG_READ | AD7193_DATA_REG | 0b01011000;

    int length = 2;

    uint32_t results [length];
    double volts [length];
    uint32_t timeb [length];
    uint8_t statusv [length];
    

    for(int i = 0; i < length; i++){

        uint32_t result = AD7193_driver(driver_args);
        uint8_t status = (uint8_t)(result & 0x000000FF);
        statusv[i] = status;
        results[i] = result;
        volts[i] = AD7193_codeToVolts(result >> 8, gain_bits, false);
        Serial.printf("%lf\n", volts[i]);
    }
    delayMicroseconds(1000);

    digitalWrite(10, HIGH);
	return true;
  
}

void setup() {
    //pinMode(LED_BUILTIN, OUTPUT);    
    SPI.begin();
}

void loop() {
    update();
}