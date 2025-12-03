#include "Arduino.h"

#include "Spi.h"
#include "SST25VF.h"

#define MemCs PC13 //chip select
#define MemWp PE1 //write protection
#define MemHold PE0 //hold

SST25VF flash;

#define DL 11
uint8_t buffer[DL]; //20130903T2046Z
void setup()
{
  Serial.begin(38400);

  SPI.setMOSI(PC12);
  SPI.setMISO(PC11);
  SPI.setSCLK(PC10);
  //SPI.begin(PC13);
  Serial.begin(38400);

  flash.begin(MemCs,MemWp,MemHold);


}

void loop() 
{
     Serial.println("writing bytes....");
  for(uint8_t i=0;i<11;i++)
  {
    flash.writeByte((uint32_t)i,i);
    Serial.print("address: ");
    Serial.println(i);
  }
  Serial.println("reading bytes....");

  long x=0;
  flash.readInit((4096UL*x));

  boolean sectorEmpty = true;
  for (int q=0; q<11; q++)
  {
    uint8_t result = flash.readNext();
    Serial.println(result,DEC);
    if (result == 0xFF)
    {

      break;
    }
  }

  flash.readFinish();
//-----------------------------------------------
  Serial.println("erasing...");
  flash.sectorErase(0);

  Serial.println("writing array...");

  for(uint8_t i=0;i<DL;i++)
  {
    buffer[i] = i;
  }

  for(uint8_t j=0;j<10;j++)
  {
    flash.writeArray(DL*j, buffer,DL);
  }

  //-------------------------------------------

  Serial.println("reading Array Written bytes....");

  for(uint8_t j=0;j<10;j++)
  {
    flash.readInit((DL*j));


    for (int q=0; q<DL; q++)
    {
      uint8_t result = flash.readNext();
      Serial.println(result);
      buffer[q] = result;

    }

    flash.readFinish();


    Serial.println("--------------------------------");
     for(uint8_t i=0;i<DL;i++)
     {
        Serial.print(" ");
        Serial.print(buffer[i]);
     }
  }
  Serial.println(".");
  flash.sectorErase(x);
  delay(200);
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
