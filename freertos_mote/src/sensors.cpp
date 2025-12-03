#if defined(MOTE4_TEENSY41) || defined(MOTE5_TEENSY41)
    #include "arduino_freertos.h"
    #include "avr/pgmspace.h"
    using namespace arduino;

    #include "queue.h"
#elif defined(MOTE_STM32)
    #include <STM32FreeRTOS.h>
#endif

#include "sensors.h"
#include "network.h"
#include "params.h"

void sensor_thread(void* args){
    sensor_handle_t sensor_list[MAX_NUM_SENSORS];
    int max_sensor_idx = 0;

    sensor_thread_arg_t* s_args = (sensor_thread_arg_t*)args;
    
    int bus_num = ((sensor_thread_arg_t*)args)->bus_num;

    while(1) {
        //check for new sensors

        //Serial.printf("Pointer to Q [THREAD]: %p\n", sensor_thread_list[bus_num].queue);
        while (xQueueReceive(sensor_thread_list[bus_num].queue, &sensor_list[max_sensor_idx], 0)) {
            Serial.println("Configured sensor!");
            Serial.printf("pin num = %d\n", sensor_list[max_sensor_idx].pin_num);
            max_sensor_idx ++;
        }

        //main sensor loop
        for (int i = 0; i < max_sensor_idx; i++) {

            //Serial.printf("Driver ptr [RECV] = %p\n", sensor_list[i].sensor_driver);

            int32_t result = sensor_list[i].sensor_driver(sensor_list[i].sensor_args);
            //int32_t result = 42;
            //Serial.printf("Result = %d\n", result);

            sensor_telemetry_t telem;
            telem.pin_num = sensor_list[i].pin_num;
            telem.data = result;

            //pass result off to network thread
            xQueueSend(telemetry_queue, &telem, 0);
        }

        //delay to keep loop time about steady
        //Serial.println("Looping sensor!");
        vTaskDelay(SENSOR_THREAD_RATE_MS / portTICK_PERIOD_MS);
    }
}

/**********************************
 * PUT SESNOR DRIVER FUNCTIONS HERE
 * ********************************/

/*
    EXAMPLE:
    should have return type int and have a single void* param
    define a parameter struct in sensors.h and then cast it in the body of your function
*/

typedef struct {
    int foo;
    uint8_t bar;
} sensor_example_args_t; //your arg structs should be in sensors.h, not here

int example_sensor_func(void* args) {
    sensor_example_args_t* s_args = (sensor_example_args_t*) args;
    int foo = s_args->foo; //s_args is a pointer, so we access it with the -> operator
    return 42;
}

int read_teensy_adc(void* args) {
    Serial.println("Reading TeensyADC");
	int analog_output = analogRead(TEENSY_INTERNAL_ADC_PIN);

    Serial.println("ending read teensy adc!");
	return analog_output;
}

int AD7193_driver(void* args){
    //NOT DONE! WIP...
    //If any problems occur during testing, they are more than likely going to be related to SPI/conversion timing!!!
    AD7193_arg_t* s_args = (AD7193_arg_t*) args;
    //Basic init stuff -- figure out which SPI bus we're using, then get and set the right pins accordingly...
    uint8_t CS_pin = (s_args->spi_bus == &SPI)? 10 : 0;
    uint8_t rdy_pin = (s_args->spi_bus == &SPI)? 12 : 1;
    pinMode(CS_pin, OUTPUT);
    digitalWrite(CS_pin, LOW);
    //ADC enters a low power mode after making a conversion, so we have to wait for it to fully power up before communicating with it to avoid problems...

    //TODO WARNING this statement is not thread safe. Make it so when integrating with MINOS
    delay(1);

    s_args->spi_bus->begin();
    s_args->spi_bus->transfer(s_args->comm_bits);
    
    uint8_t reg_id = s_args->comm_bits & 0b00111000;
    if(s_args->comm_bits & AD7193_REG_READ){
        //Reading data from one of the ADC's registers...
        //Readable registers in the AD7193 have one of two sizes: 8-bit, and 24-bit. The 8-bit registers are STATUS, ID, and GPOCON, with all others being 24-bit...
        if(reg_id == AD7193_STATUS_REG || reg_id == AD7193_ID_REG || reg_id == AD7193_GPOCON_REG)
            return s_args->spi_bus->transfer(0);
        else{
            //Read operations from the data register trigger a fresh conversion in the ADC, so we have to wait for the result to settle before reading...
            //The ADC signals that it's ready to output a reading by bringing its D_OUT pin low (without being triggered by the clock), so we wait for this to happen before proceeding...
            if(reg_id == AD7193_DATA_REG){
                while(1){
                    if(digitalRead(rdy_pin) == LOW)
                        break;
                    delay(1);
                }
            }
            uint32_t sensor_return = 0;
            //Bits are delivered to us going from MSB to LSB...
            //Include logic here to account for whether or not the status register bits are being appended!!!!
            sensor_return |= s_args->spi_bus->transfer(0) << 16;
            sensor_return |= s_args->spi_bus->transfer(0) << 8;
            sensor_return |= s_args->spi_bus->transfer(0);
            return sensor_return;
        }
        //End of READ if-block...
    }
    else{
        //Writing data to one of the ADC's registers...
        //Only one of the AD7193's writeable registers has 8-bit size -- the rest are 24-bit...
        if(reg_id == AD7193_GPOCON_REG)
            s_args->spi_bus->transfer((uint8_t)s_args->write_payload);
        else{
            //We transfer the bits going from MSB to LSB...
            s_args->spi_bus->transfer((uint8_t)(s_args->write_payload >> 16));
            s_args->spi_bus->transfer((uint8_t)(s_args->write_payload >> 8));
            s_args->spi_bus->transfer((uint8_t)s_args->write_payload);
        }
        return 0;
        //End of WRITE else-block
    }
    //Deselect device to avoid spurious writes...
    digitalWrite(CS_pin, HIGH);
}

double AD7193_codeToVolts(uint32_t code, uint16_t currentGainSetting, bool unipolar){
    //The following gain codes appear on page 28 of the AD7193 data sheet! (I have no idea what "Reserved" means there or what those codes are for, hopefully it's not anything important >_<!!)
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
        return ((double)(code * VREF)) / ((double)(AD7193_CODE_EXP_TERM * gain));

    //If the above conditional statement fails, then the ADC is in bipolar mode! Following is the formula for the bipolar case...
    return ( ( (double)code / ((double)AD7193_CODE_EXP_TERM / 2) ) - 1 ) / ( (double)gain / (double)VREF );
}

//Use the following function if configs need to be adjusted before making a read. Otherwise, use the raw driver above.
int AD7193_wrapper(void* args){
    AD7193_wrapper_arg_t* configs = (AD7193_wrapper_arg_t*)args;
    //Form a configuration bit-string...
    
    uint16_t gain_bits;
    switch(configs->pga_gain){
        case 1:     gain_bits = 0x0;    break;
        case 8:     gain_bits = 0x03;   break;
        case 16:    gain_bits = 0x04;   break;
        case 32:    gain_bits = 0x05;   break;
        case 64:    gain_bits = 0x06;   break;
        case 128:   gain_bits = 0x07;   break;
        default:    gain_bits = 0x0;
    }

    uint16_t channel_bits;
    if (configs->internal_temp) {
        channel_bits = 8;
    } else {
        channel_bits = (configs->channel - 1);
    }

    //differential bit is flipped, this is because it is a "psuedo" bit in the datasheet. I.e, if the bit is set, take differential against AINCOM instead of an input
    uint32_t config_write_payload = 0x0 | (!configs->differential << AD7193_DIFFERENTIAL_BIT) | (AD7193_CHANNEL_SHIFT << channel_bits) | (gain_bits << AD7193_PGA_BIT) ;
    Serial.println(config_write_payload);

    AD7193_arg_t driver_args;
    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_CONFIG_REG;
    driver_args.write_payload = config_write_payload;

    switch (configs->spi_num) {
        case 0: driver_args.spi_bus = &SPI; break;
        case 1: driver_args.spi_bus = &SPI1; break;
        default: driver_args.spi_bus = &SPI;
    }
    

    AD7193_driver((void*)&driver_args);
    //Just reuse the old structure to avoid waste. The payload section doesn't need to be changed since the driver ignores it on read operations.
    driver_args.comm_bits = AD7193_REG_READ | AD7193_DATA_REG;

    int result = AD7193_driver((void*)&driver_args);
    Serial.println(result);

    double units;
    if (!configs->internal_temp) {
        units = AD7193_codeToVolts(result, gain_bits, false);
    } else {
        units = (result - 0x800000)/2815.0 - 273;
    }
    
    return units * 1000; //integral number of millivolts
}