#include "Arduino.h"
#include "SPI.h"
#include "pins_arduino.h"

/*
  SPI_loopback

  Test the loopback transfer on SPI.
  A single SPI instance is used, the default one.
  MISO pin must be externally connected to MOSI pin.
  To test the transfer on different SPI instance, just redefine SPI pins
  as follows:
  #define MOSI_PIN  XXX
  #define SCK_PIN     XXX
  #define MISO_PIN  XXX
   #define CS_PIN      XXX

  Test behavior:
  A test string is sent 9 times and compared to the Rx frame.
  The LED remains ON when test ends successfully
  The LED remains OFF in case of error.
  This example code is in the public domain.
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN PNUM_NOT_DEFINED
#warning "LED_BUILTIN is not defined."
#endif

#define MOSI_PIN  PB15
#define MISO_PIN  PB14
#define SCK_PIN   PB13
#define CS_PIN    PB11

uint8_t buffer_tx[] = "Thequickbrownfoxjumpsoverthelazydog\0";
#define BUF_SIZE  sizeof(buffer_tx)
uint8_t buffer_rx[BUF_SIZE] = {};

/* SPI transfer loop nb */
#define TEST_LOOP_NB 9

void setup() {
  SPI.setMOSI(MOSI_PIN);
  SPI.setMISO(MISO_PIN);
  SPI.setSCLK(SCK_PIN);
  // Don't set CS_PIN to have Software ChipSelect management
  // Instead, CS_PIN is passed as argument to SPI.transfer(CS_PIN)
  // SPI.setSSEL(CS_PIN);

  /* the SPI pin configuration is done by the SPI.begin */
   SPI.begin(CS_PIN);

   while(1) {
    delay(1);
   }
}

void loop() {
  uint8_t tested_ok = 0; // test result

  for (uint8_t received = 0; received < TEST_LOOP_NB; received++) {
    SPI.transfer(CS_PIN, buffer_tx, buffer_rx, BUF_SIZE, SPI_LAST);

    /* compare what is received to what was sent */
    if (!memcmp(buffer_tx, buffer_rx, BUF_SIZE)) {
      /* this transfer passed */
      tested_ok++;
    }
    memset(buffer_rx, 0, BUF_SIZE);
  }
  
  /* display test result */
  pinMode(LED_BUILTIN, OUTPUT);   // Configure LED pin, for test result
  digitalWrite(LED_BUILTIN, LOW); // start with led off
  if (tested_ok == TEST_LOOP_NB) {
      /* success */
      digitalWrite(LED_BUILTIN, HIGH);
  } else {
      /* error. Please verify MISO is externally connected to MOSI */
      digitalWrite(LED_BUILTIN, LOW);
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