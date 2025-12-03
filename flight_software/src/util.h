#ifndef UTIL_H_
#define UTIL_H_

#include "Arduino.h"
#include "minos_util.h"
#include "MinosSensor.h"
#include "network.h"
#include "AdcChannel.h"
#include "AD7193.h"
#include "SolenoidDriver.h"
#include "SDcard.h"
#include "Watchdog.h"
#include "LoopTimer.h"
#include <map>

#ifdef FLIGHT_COMPUTER
  #include "VoltMonitor.h"
  #include "BNO.h"
  #include "BNOChannel.h"
  #include "BMP388.h"
  #include "BMPChannel.h"
  #include "Flash.h"
  #include "internalFlash.h"
  #include "BatteryManager.h"
  #include "GPS.h"
  #include "GPSChannel.h"

  void flight_sensors(SensorMap& sensors, ActuatorMap& actuators);

#endif

#ifdef HITL
  #include "HitlManager.h"
#endif


#endif
