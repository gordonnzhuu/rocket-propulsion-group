#include "Arduino.h"
#include "Adafruit_BMP3XX.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <Wire.h>
#include "pins_arduino.h"

#include <SPI.h>
#include <Wire.h>
#include "BNO055.h"
#include "BMP388.h"
#include "BNO.h"

// Adafruit_BMP3XX bmp;
// BNO055 bno;
// TwoWire two(PB7,PB6);
// BNO055::BNO055_opmode_t mode = BNO055::OPERATION_MODE_IMUPLUS;

volatile bool sensorcheck;
BNO bno1(two);
BMP388 bmp1(two);

void setup()
{
  //pinMode(PB6, INPUT_PULLUP);
 // pinMode(PB7, INPUT_PULLUP);
    // sensorcheck = bmp.begin_I2C(0x77, &two);
    // Serial.begin(9600);
    // //
    // Wire.setSCL(PB6);
    // Wire.setSDA(PB7);
    // bno.Initialize(0x28, BNO055::OPERATION_MODE_IMUPLUS, two);

    // bno.setExtCrystalUse(true);
      bmp1.init();
    bno1.init();
  
}

void loop() 
{
    // volatile float p = bmp.readPressure();
    // volatile bool t = sensorcheck;
    // Serial.print(p);
    // delay(1000);
    // //
    // BNO055_calibration_state_t other_thingy = bno.getCalibration();

    // BNO055_measurment_data_t thingy;
    // thingy = bno.getFullMeasurment(false, true);
    
    bmp1.update();
    bmp1.get_data();

    bno1.update();
    bno1.get_data();
    // __NOP();


   
      __NOP();
}



extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}