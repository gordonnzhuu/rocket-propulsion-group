#include "BNO.h"

BNO055 accel;
bool BNO::init(TwoWire i2c)
{
  
  BNO055::BNO055_opmode_t mode = BNO055::OPERATION_MODE_IMUPLUS;
  TwoWire test = two;
  if(accel.Initialize(0x28, mode,test)) {
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    accel.setExtCrystalUse(true);
    accel.setAccelerometerRange(BNO055::BNO_acc_range_t::G_16);
    accel.setSensorOffsets(BNO::config);
  return accel.isFullyCalibrated();
  }
  else {
    return false;
  }

  return false;

}

void BNO::set_config(uint8_t *config)
{
  memccpy(BNO::config, config, 0, 22);
}

bool BNO::get_config(uint8_t *config)
{
  return accel.getSensorOffsets(config);
}


bool BNO::update()
{
  BNO::data = accel.getFullMeasurment(false, true);
  __NOP();
    return true; 
}

uint32_t BNO::get_data()
{
    return (uint32_t) BNO::data.LinearAcceleration.X; // not sure if we want pass the whole BNO055_measurment_data_t or not
}

uint32_t BNO::get_data(char axis)
{
  switch (axis)
  {
    case 'x':
      return (uint32_t) BNO::data.LinearAcceleration.X;
    case 'y':
      return (uint32_t) BNO::data.LinearAcceleration.Y;
    case 'z':
      return (uint32_t) BNO::data.LinearAcceleration.Z;
    default:
      return (uint32_t) BNO::data.LinearAcceleration.X;
  }
}

BNO055_measurment_data_t BNO::get_data_struct()
{
  return BNO::data;
}
