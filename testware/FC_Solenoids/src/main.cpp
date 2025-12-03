#include "Arduino.h"
#include "SPI.h"
#include "pins_arduino.h"

void setup() {
    pinMode(PD8, OUTPUT);
    pinMode(PD9, OUTPUT);
    pinMode(PD10, OUTPUT);
    pinMode(PD11, OUTPUT);

    pinMode(PD12, OUTPUT);
    pinMode(PD13, OUTPUT);
    pinMode(PD14, OUTPUT);
    pinMode(PD15, OUTPUT);
}

void loop() {
    //test each solenoid
    digitalWrite(PD8, HIGH);
    delay(500);
    digitalWrite(PD8, LOW);

    digitalWrite(PD9, HIGH);
    delay(500);
    digitalWrite(PD9, LOW);

    digitalWrite(PD10, HIGH);
    delay(500);
    digitalWrite(PD10, LOW);

    digitalWrite(PD11, HIGH);
    delay(500);
    digitalWrite(PD11, LOW);

    digitalWrite(PD12, HIGH);
    delay(500);
    digitalWrite(PD12, LOW);

    digitalWrite(PD13, HIGH);
    delay(500);
    digitalWrite(PD13, LOW);

    digitalWrite(PD14, HIGH);
    delay(500);
    digitalWrite(PD14, LOW);

    digitalWrite(PD15, HIGH);
    delay(500);
    digitalWrite(PD15, LOW);
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