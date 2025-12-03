#include <Arduino.h>

#define TACH_B 16
#define TACH_A 17

//https://www.ti.com/product/LM2907-N/part-details/LM2907M-8/NOPB

void setup() {
    Serial.printf("Testing Tachometer");
    pinMode(TACH_A, INPUT);
    pinMode(TACH_B, INPUT);

    Serial.begin(9600);
}
void read_tach(int tach_pin){

    int sensor_value = analogRead(tach_pin);  
    float voltage = sensor_value * (1.65 / 1023.0);  // 1.65 V is from calculations on motev5 pcb pdf. 1023 is from Teensy documentation
    float frequency = voltage * 10000.0;  // 10kHz/V transfer function from datasheet
    Serial.printf("Tach pin %d V: %f \n", tach_pin, voltage);

}
void loop() {

    read_tach(TACH_A);
    read_tach(TACH_B); 
    delay(1000);
}