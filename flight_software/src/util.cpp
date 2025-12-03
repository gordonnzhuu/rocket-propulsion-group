#include "util.h"

#ifdef FLIGHT_COMPUTER
void flight_sensors(SensorMap& sensors, ActuatorMap& actuators) {
    //Configures sensors for flight computer if it power cycles after launch
    //Pressure Transducers
    sensors[0] = new AdcChannel(0, Adc0); // POTR
    sensors[1] = new AdcChannel(1, Adc0); // PFTR
    sensors[2] = new AdcChannel(2, Adc0); // POIM
    sensors[3] = new AdcChannel(3, Adc0); // PFIM
    sensors[4] = new AdcChannel(4, Adc0); // PMCC
    Adc0.set_pga_gain(1);

    //Thermocouples
    sensors[10] = new AdcChannel(10, Adc1); // TOTR
    sensors[18] = new AdcChannel(18, Adc1); // TMP1
    Adc1.set_pga_gain(128);

    //Main loop timer
    sensors[80] = new LoopTimer(0.1);

    //Rail voltage monitors
    sensors[71] = new VoltMonitor(71); // 12VM
    sensors[72] = new VoltMonitor(72); // 3V3M
    sensors[73] = new VoltMonitor(73); // 5V0M

    //BMP388 Pressure and Temperature Sensor
    sensors[40] = new BMPChannel(BMP388::BMP_ALT);
    sensors[41] = new BMPChannel(BMP388::BMP_P);
    sensors[42] = new BMPChannel(BMP388::BMP_TMP);

    //BNO055 IMU
    sensors[50] = new BNOChannel(BNO::IMU_ACCEL_X);
    sensors[51] = new BNOChannel(BNO::IMU_ACCEL_Y);
    sensors[52] = new BNOChannel(BNO::IMU_ACCEL_Z);

    //GPS not sure which pins to use???
    sensors[60] = new GPSChannel(GPS::GPS_LAT);
    sensors[61] = new GPSChannel(GPS::GPS_LON);
    sensors[62] = new GPSChannel(GPS::GPS_ALT);
    sensors[63] = new GPSChannel(GPS::GPS_SPEED);
    sensors[64] = new GPSChannel(GPS::GPS_ANGLE);
 }
 #endif

