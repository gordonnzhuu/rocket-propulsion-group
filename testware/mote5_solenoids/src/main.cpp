#include <Arduino.h>

//solenoid pins1
#define SOLENOID_1_PIN 5
#define SOLENOID_2_PIN 6
#define SOLENOID_3_PIN 7
#define SOLENOID_4_PIN 8
#define SOLENOID_5_PIN 9
#define SOLENOID_6_PIN 29
#define SOLENOID_7_PIN 28

//change here
#define TEST_PIN 7

bool states[7] = {false, false, false, false, false, false, false};

void toggle_solenoid(int solenoid_pin) {
    int teensy_pin = -1;

    switch (solenoid_pin) {
        case 1:
            teensy_pin = SOLENOID_1_PIN;
            break;
        case 2:
            teensy_pin = SOLENOID_2_PIN;
            break;
        case 3:
            teensy_pin = SOLENOID_3_PIN;
            break;
        case 4:
            teensy_pin = SOLENOID_4_PIN;
            break;
        case 5:
            teensy_pin = SOLENOID_5_PIN;
            break;
        case 6:
            teensy_pin = SOLENOID_6_PIN;
            break;
        case 7:
            teensy_pin = SOLENOID_7_PIN;
            break;
    }

    if (teensy_pin == -1) {
        return;
    }

    states[solenoid_pin - 1] = !states[solenoid_pin - 1];
    pinMode(teensy_pin, OUTPUT);    
    Serial.printf("Setting teensy pin %d to %d\n", teensy_pin, states[teensy_pin]);
    digitalWrite(teensy_pin, states[solenoid_pin - 1]);
    
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);    
    Serial.begin(9600);
}

void loop() {
    /*
    Serial.printf("Enter solenoid: ");

    while(Serial.available() == 0) {}
    uint8_t pin = Serial.read() - '0';
    delay(50);

    toggle_solenoid(pin);
    Serial.println(pin);
    
    delay(50);
    */

   digitalWrite(LED_BUILTIN, states[TEST_PIN - 1]);
   toggle_solenoid(TEST_PIN);
   delay(3000);
}