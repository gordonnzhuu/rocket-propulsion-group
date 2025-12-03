#include "BNO.h"

BNO055 accel;

volatile bool bno_drdy = false;

bool BNO::init(TwoWire &i2c)
{
 // NDOF if you want all of the data but only at 4g and 100hz
  //ACCONLY if you only want acceleration and 330hz
  //ACCGYRO if you want acceleration and angular velocity at 150hz
  mode = BNO055::OPERATION_MODE_ACCONLY; // 
  if(accel.Initialize(0x28, mode, i2c)) {
    accel.setMode(BNO055::OPERATION_MODE_CONFIG);
    accel.setSensorOffsets(config); // These probably need to be tuned so that gravity is closer to 9.81 m/s^2
    accel.setAccelerometerRange(BNO055::BNO_acc_range_t::G_16);
    accel.setAccelerometerBW(BNO055::BNO_acc_BW_t::Hz_1000);

    // accel.setGyroscopeBW(BNO055::BNO_gyr_BW_t::Hz_523);
    // accel.setGyroscopeRange(BNO055::BNO_gyr_range_t::dps_1000); // hopefully the rocket doesn't spin faster than 150 rpm -  blame Thomas if it does 
    // Turns out we will never know how fast the rocket will spin
    accel.setMode(mode);
    accel.setExtCrystalUse(true);

    volatile BNO055::BNO_acc_range_t range = accel.getAccelerometerRange();  
    
    initialized = true;
    return true;
  }
  initialized = false;
  return false;

}

// Call this after seperation to better data on the way down 
bool BNO::IMUinit(TwoWire &i2c)
{
  mode = BNO055::OPERATION_MODE_NDOF; //
  if(initialized){
    accel.setMode(BNO055::OPERATION_MODE_CONFIG); 
    accel.setMode(mode);
    if(accel.getAccelerometerRange() == BNO055::BNO_acc_range_t::G_04) {
      initialized = true;
      return true;
    }
    initialized = false;
  }
  return false;
}

bool BNO::getinitialized()
{
  return initialized;
}

void BNO::set_config(uint8_t *config)
{
  memccpy(config, config, 0, 22);
}

bool BNO::get_config(uint8_t *config)
{
  return accel.getSensorOffsets(config);
}

uint32_t BNO::get_accel_magnitude()
{
  return accel_magnitude;
}

bool BNO::update()
{
    if (undersample_count >= num_undersample) {
      uint32_t dtb = millis();
      if(mode == BNO055::OPERATION_MODE_NDOF) {
        data = accel.getFullMeasurment();
      } else {
        imu::Vector<3> xyz = accel.getVector(BNO055::SENSOR_ACCELEROMETER);
        //imu::Vector<3> gyro = accel.getVector(BNO055::SENSOR_GYROSCOPE);

        /* 1m/s^2 = 100 LSB  so this is in cm/s^2 */
        data.Acceleration.X = (int16_t)(xyz[0] * 100);
        data.Acceleration.Y = (int16_t)(xyz[1] * 100);
        data.Acceleration.Z = (int16_t)(xyz[2] * 100);
        accel_magnitude = sqrt(pow(data.Acceleration.X, 2) + pow(data.Acceleration.Y, 2) + pow(data.Acceleration.Z, 2));

        /* 1dps = 16 LSB so this is in dps */
        //data.AngularVelocity.X = (uint32_t)(gyro[0] * 16);
        //data.AngularVelocity.Y = (uint32_t)(gyro[1] * 16);
        //data.AngularVelocity.Z = (uint32_t)(gyro[2] * 16);
      }
      volatile uint32_t dtb2 = millis() - dtb;
      undersample_count = 0;
    } else {
      undersample_count += 1;
    }
  __NOP();
  return true; 
  
}


uint32_t BNO::get_data(channel_config_t axis)
{
  switch (axis)
  {
    case IMU_ACCEL_X:
      return (uint32_t) data.Acceleration.X;
    case IMU_ACCEL_Y:
      return (uint32_t) data.Acceleration.Y;
    case IMU_ACCEL_Z:
      return (uint32_t) data.Acceleration.Z;
    case IMU_ANGVEL_X:
      return (uint32_t) data.AngularVelocity.X;
    case IMU_ANGVEL_Y:
      return (uint32_t) data.AngularVelocity.Y;
    case IMU_ANGVEL_Z:
      return (uint32_t) data.AngularVelocity.Z;
    case IMU_MAG_X:
      return (uint32_t) accel_magnitude;
    case IMU_MAG_Y:
      return (uint32_t) data.MagneticField.Y;
    case IMU_MAG_Z:
      return (uint32_t) data.MagneticField.Z;
    case IMU_GRAV_X:
      return (uint32_t) data.GravityVector.X;
    case IMU_GRAV_Y:
      return (uint32_t) data.GravityVector.Y;
    case IMU_GRAV_Z:
      return (uint32_t) data.GravityVector.Z;
    case IMU_LIN_X:
      return (uint32_t) data.LinearAcceleration.X;
    case IMU_LIN_Y:
      return (uint32_t) data.LinearAcceleration.Y;
    case IMU_LIN_Z:
      return (uint32_t) data.LinearAcceleration.Z;
    default:
      return (uint32_t) data.LinearAcceleration.X; 
  }
}

BNO055_raw_measurment_data_t BNO::get_data_struct()
{
  return data;
}
