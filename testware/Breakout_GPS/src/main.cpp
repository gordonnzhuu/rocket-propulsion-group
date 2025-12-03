#include <Arduino.h>

// Make sure to install the adafruit GPS library from https://github.com/adafruit/Adafruit-GPS-Library
#include <Adafruit_GPS.h>   //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SoftwareSerial.h> //Load the Software Serial Library. This library in effect gives the arduino additional serial ports
#include <HardwareSerial.h>

#define PIN_SERIAL1_TX PB7
#define PIN_SERIAL1_RX PB6
#define GPS_USART_1 Serial1
#define GPS_BYPASS true
#define DEBUG true

#ifndef Serial1
HardwareSerial Serial1(PIN_SERIAL1_TX, PIN_SERIAL1_RX);
#endif

Adafruit_GPS GPS(&GPS_USART_1); // Create GPS object

String NMEA1; // We will use this variable to hold our first NMEA sentence
String NMEA2; // We will use this variable to hold our second NMEA sentence
char c;       // Used to read the characters spewing from the GPS module

void clearGPS()
{ // Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
  while (!GPS.newNMEAreceived())
  {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}

void readGPS()
{             // This function will read and remember two NMEA sentences from GPS
  clearGPS(); // Serial port probably has old or corrupt data, so begin by clearing it all out
  while (!GPS.newNMEAreceived())
  {                 // Keep reading characters in this loop until a good NMEA sentence is received
    c = GPS.read(); // read a character from the GPS
  }
  GPS.parse(GPS.lastNMEA()); // Once you get a good NMEA, parse it
  NMEA1 = GPS.lastNMEA();    // Once parsed, save NMEA sentence into NMEA1
  while (!GPS.newNMEAreceived())
  { // Go out and get the second NMEA sentence, should be different type than the first one read above.
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();

  Serial.println(NMEA1);
  Serial.println(NMEA2);
  Serial.println("");
}

void setup()
{
  Serial.begin(115200); // Turn on the Serial Monitor
  while (!Serial)
    delay(10);
  GPS.begin(9600);                              // Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D");            // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    // 5 Hz update rate
  // delay(1000);                                  // Pause
}

void loop() // run over and over again
{
  readGPS();  // This is a function we define below which reads two NMEA sentences from GPS
}