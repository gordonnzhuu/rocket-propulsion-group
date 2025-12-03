#include "Arduino.h"

//make sure stm32duino works properly
#include <Device.h>
#include <BMP388_DEV.h>                           // Include the BMP388_DEV.h library

TwoWire i2c(PB7, PB6);
BMP388_DEV bmp388(i2c);                           // Instantiate (create) a BMP388_DEV object and set-up for I2C operation (address 0x77)

volatile boolean dataReady = false;

void interruptHandler()                           // Interrupt handler function
{
  dataReady = true;                               // Set the dataReady flag
}

void setup() 
{
  bmp388.begin();                                   // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.enableInterrupt();                         // Enable the BMP388's interrupt (INT) pin
  bmp388.setPresOversampling(OVERSAMPLING_X2);
  bmp388.setTempOversampling(OVERSAMPLING_SKIP);
  attachInterrupt(PD2, interruptHandler, RISING);   // Set interrupt to call interruptHandler function on D2
  bmp388.setTimeStandby(TIME_STANDBY_10MS);         // Set the standby time to 1.3 seconds
  bmp388.startNormalConversion();                   // Start BMP388 continuous conversion in NORMAL_MODE  
}

void loop() 
{
  float temperature, pressure, altitude;

  if (dataReady)
  {  
    bmp388.getMeasurements(temperature, pressure, altitude);      // Read the measurements
    __NOP();
    dataReady = false;                            // Clear the dataReady flag
  }   
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