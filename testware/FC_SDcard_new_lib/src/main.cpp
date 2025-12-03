/*
 * Example use of two SPI ports on an STM32 board.
 * Note SPI speed is limited to 18 MHz.
 */
#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"
//#error See new Version 2 STM32 example
// set ENABLE_EXTENDED_TRANSFER_CLASS non-zero to use faster EX classes



SdFat sd1;
// SdFatEX sd1;
const uint8_t SD1_CS = PA10;  // chip select for sd1

#define SD_CS_PIN PA10
static SPIClass sd_spi( PC12, PC11, PC10, PNUM_NOT_DEFINED); 
#define SD2_CONFIG SdSpiConfig(PNUM_NOT_DEFINED, SHARED_SPI, SD_SCK_MHZ(18), &sd_spi)
 

// Use second SPI port
// SPIClass SPI_2(2);
// SdFat sd2(&SPI_2);
// SdFatEX sd2(&SPI_2);

//const uint8_t SD2_CS = PB12;   // chip select for sd2

const uint8_t BUF_DIM = 100;
char buf[BUF_DIM];

const uint32_t FILE_SIZE = 100;
const uint16_t NWRITE = FILE_SIZE/BUF_DIM;
//------------------------------------------------------------------------------
// print error msg, any SD error codes, and halt.
// store messages in flash
#define errorExit(msg) errorHalt(F(msg))
#define initError(msg) initErrorHalt(F(msg))
//------------------------------------------------------------------------------
void setup() {
  //Serial.begin(9600);
  // Wait for USB Serial

  // fill buffer with known data
  for (size_t i = 0; i < sizeof(buf); i++) {
    buf[i] = '3';
  }
  sd_spi.setMISO(PC11);
  sd_spi.setMOSI(PC12);
  sd_spi.setSCLK(PC10);

  pinMode(SD1_CS, OUTPUT);
  digitalWrite(SD1_CS, HIGH);
  delay(1);
  digitalWrite(SD1_CS, LOW);
  // initialize the first card
  if (!sd1.begin(SD2_CONFIG)) {
    while (1)
    {
      if(sd1.begin(SD2_CONFIG)){
        break;
      }
      /* code */
    }
    
    sd1.initError("sd1:");
  }
  // create Dir1 on sd1 if it does not exist
  // if (!sd1.exists("/Dir1")) {
  //   if (!sd1.mkdir("/Dir1")) {
  //     sd1.errorExit("sd1.mkdir");
  //   }
  // }


  // create or open /Dir1/test.bin and truncate it to zero length
  SdFile file1;

  //Serial.println(F("Writing test.bin to sd1"));

  // write data to /Dir1/test.bin on sd1
  if (!file1.open("test.txt", O_RDWR | O_CREAT )) {
    sd1.errorExit("file1");
  }
  uint32_t dt = millis();
  for (uint16_t i = 0; i < NWRITE; i++) {
    if (file1.write(buf, sizeof(buf)) != sizeof(buf)) {
      sd1.errorExit("sd1.write");
    }
  }
  uint32_t pos = file1.curPosition();
  file1.close();
  
  for (size_t i = 0; i < sizeof(buf); i++) {
    buf[i] = '4';
  }
  if (!file1.open("test.txt", O_RDWR )) {
    sd1.errorExit("file1");
  }
  //file1.seekSet(0);
  
  file1.seekCur(pos);
  //file1.seekEnd(0);
  for (uint16_t i = 0; i < NWRITE; i++) {
      if (file1.write(buf, sizeof(buf)) != sizeof(buf)) {
      sd1.errorExit("sd1.write");
    }
  }
 // file1.flush();
  file1.close();
  
  volatile uint32_t dt2 = millis() - dt;
  // close test.bin
  dt2 = dt2;

}
//------------------------------------------------------------------------------
void loop() {}

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