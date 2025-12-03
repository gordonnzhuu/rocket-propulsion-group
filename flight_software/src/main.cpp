#include "util.h"

#ifdef FLIGHT_COMPUTER
  #include "StateMachine.h"
  #include "ApogeeDetector.h"
#endif

#ifdef BUILD_LORA
  #include <Lora.h> // Lora Lib
#endif


#ifndef CS_PIN_CONTROLLED_BY_USER
  #define CS_PIN_CONTROLLED_BY_USER 79
#endif

/**
 * Make std::maps (C++ version of dictionaries or hashmaps) to store all currently used sensors and actuators
 * The key is currently the "virtual pin number" used by MoTE, but this should be changed.
 * @todo Make the map key a P&ID key
*/
SensorMap sensors;
ActuatorMap actuators;

SensorSerialMap sensors_serial;

/**
 * Define all the hardware specific to the board here.
 * It is ok to hardcode stuff specific to the hardware for now.
 * At some point we will have to make it so that this is configurable though
 * To run this on both DaeGSE and DaeFlight.
*/

#ifndef CS_PIN_CONTROLLED_BY_USER
  #define CS_PIN_CONTROLLED_BY_USER 79
#endif

#ifdef FLIGHT_COMPUTER
  HardwareSerial gps_serial(PA1, PA0);
  TwoWire sensor_i2c(PB7, PB6);
  SPIClass adc_spi(PB15, PB14, PB13, CS_PIN_CONTROLLED_BY_USER);
  FlightStateMachine state_machine;
  #define ADC0_CS PB11
  #define ADC1_CS PB12
  #define ADC0_SPI adc_spi
  #define ADC1_SPI adc_spi
  #define ADC0_DRDY PB14
  #define ADC1_DRDY PB14
  
  #define LED_PIN PE8
  bool ledState = false;
  uint32_t t_previous = 0;
  uint32_t blink_interval = 1000;

  GPS gps(gps_serial);
  BMP388 bmp(sensor_i2c);
  BNO bno;

  //Flash flash(PC13);
  internalFlash iFlash;

  // I would like a word with whoever made this in MILLIMETERS PER SECOND
  // And other perfectly normal units.
  ApogeeDetector apogee_detector(0.05, 0.005, 200, -10e3);

  SPIClass sd_lora_spi(PC12, PC11, PC10, CS_PIN_CONTROLLED_BY_USER);
  
#elif defined(GSE_COMPUTER)
  #define ADC0_CS 10
  #define ADC1_CS 0
  #define ADC0_SPI SPI
  #define ADC1_SPI SPI1
  #define ADC0_DRDY 12
  #define ADC1_DRDY 1
#endif

#ifdef BUILD_LORA
  Lora lora(SX1262_SS, SX1262_DIO1, SX1262_NRST, SX1262_BUSY, sd_lora_spi);
#endif

#ifdef HITL
  HitlManager hitl;
#endif

AD7193 Adc0(ADC0_CS, ADC0_SPI, ADC0_DRDY);
AD7193 Adc1(ADC1_CS, ADC1_SPI, ADC1_DRDY);
SDcard sdcard;
Watchdog watchdog;


volatile bool sdcheck = false;
volatile bool flashcheck = false;
volatile bool bnocheck = false;
volatile bool bmpcheck = false;
volatile bool loracheck = false;

void setup() {  
  //sd_lora_spi.setClockDivider(SPI_CLOCK_DIV128);
  // Assert all the highs
  #ifdef FLIGHT_COMPUTER
  pinMode(SX1262_SS, OUTPUT);
  digitalWrite(SX1262_SS, HIGH);
  pinMode(SD1_CS, OUTPUT);
  digitalWrite(SD1_CS, HIGH);
  delay(10);
  #endif

  sdcheck = sdcard.init();
  volatile int mote_num = sdcard.getMoteNum(); // previously uint_8t, -1 int would wrap to 255 since the getMoteNum() returns an int type

  #ifdef HITL
    hitl.init();
  #endif

  /**
   * Set up initial valve states.
   * "Temporary, want to read this from config eventually."
   *    -- T. Evdokimov, February 2024
  */
  #ifdef FLIGHT_COMPUTER
    //iFlash.init();
    //iFlash.setFlightState(IDLE);
    volatile uint8_t num = iFlash.readFlightSensors(sensors, actuators);
    flight_fsm_e flight_state = sdcard.readFlightState();
    state_machine.init(flight_state);
    
    #define NUM_SOLENOIDS 8
    #define SOLENOID_PINS {PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15};
    #define SOLENOID_STATES {false, false, false, false, false, false, false, false};
  #elif defined(GSE_COMPUTER)
    #define NUM_SOLENOIDS 7
    // #define SOLENOID_PINS {5, 6, 7, 8, 9, 28, 29}
    #define SOLENOID_PINS {7, 7, 7, 7, 7, 7, 7}
    // VQDA (mote 2, solenoid 2) is N/O in software.
    // VXBP (mote 2, solenoid 4) is N/O in software.
    #define SOLENOID_STATES {false, false, mote_num == 2, false, mote_num == 2, false, false}
  #endif

  uint32_t pins[] = SOLENOID_PINS;
  uint32_t states[] = SOLENOID_STATES;

  for (int i = 0; i < NUM_SOLENOIDS; i++) {
    actuators[i] = new SolenoidDriver(pins[i], states[i]);
    actuators[i]->init();
  }

  #ifdef FLIGHT_COMPUTER
    actuators[30] = new BatteryManager();
    actuators[30]->init();
    actuators[30]->set_state(BatteryManager::BATTERY);
  #endif

  sdcard.writeActuators(actuators);

  //set up network
  #ifdef FLIGHT_COMPUTER
    pinMode(LED_PIN, OUTPUT);
    //volatile uint16_t number = EEPROM.length();
    
    // No network in flight
    // if(!flight_state)
    //   init_motenet();
    init_motenet();
    ledState = true;
    digitalWrite(LED_PIN, ledState);
    t_previous = millis();

    bnocheck = bno.init(sensor_i2c);
    bmpcheck = bmp.init();
    gps.init();
    //flashcheck = flash.init();
  #elif defined(GSE_COMPUTER)
    init_motenet();
  #endif

  #ifdef BUILD_LORA
    loracheck = lora.init();
  #endif
  
  //set up ADC
  Adc0.init();
  Adc1.init();

  for (auto sensor : sensors) {
    sensor.second->init();
  }

  #if FLIGHT_COMPUTER
    // See StateMachine.cpp
  #else
    if (mote_num == 2) {
      watchdog.add_watchdog_action(2, false); //VQDA
    }
    watchdog.set_watchdog_ttl(3 * 60 * 1000); // 3 minutes
    watchdog.set_watchdog_enabled(false);
  #endif
  Adc0.update();
  delay(1);
  Adc1.update();
  delay(1);
}

void loop() {
  volatile uint32_t t0 = millis();

  uint32_t time = millis();
  //update based on incoming data;  
  rx_motenet(sensors, actuators);

  watchdog.update(actuators);

  #ifdef HITL
    hitl.update();
  #endif

  Adc0.update();
  delayMicroseconds(1);
    Adc1.update();

  #ifdef FLIGHT_COMPUTER
    bmp.update();
    bno.update();
    gps.update();
  #endif
    
  for (auto sensor : sensors) {
    sensor.second->update();
  }

  for (auto actuator : actuators) {
    actuator.second->update();
  }
  
  if(sdcheck){
    volatile bool sd_wa_check = sdcard.writeDataAll(sensors, actuators);
  }

  #ifdef BUILD_LORA
  if(loracheck)
    lora.send_data(sensors);
  #endif
  
  #ifdef FLIGHT_COMPUTER
  //blink the LED every second when running
    uint32_t t_current = millis();
    uint32_t dt = t_current - t_previous;
    if(dt >= blink_interval){ 
      t_previous = t_current;
      ledState = !ledState; 
      digitalWrite(LED_PIN, ledState);
    }
    // Only save sensors 500 ms after last config packet
    if(iFlash.savedSensors == false && (millis() - iFlash.getTimeout() > 800)){
      iFlash.saveSensors();
    }
    state_machine.update(sensors, actuators);
  #endif
  // volatile flight_fsm_e out = iFlash.readFlightState();
  //send data
  tx_motenet(sensors, actuators);

  volatile uint32_t t1 = millis() - t0;
}

/**
 * Set the STM32 to use it's internal 8MHz oscillator.
 * The flight computer allegedly has an external 24MHz oscillator.
 * Don't worry about it.
*/
#ifdef FLIGHT_COMPUTER
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
#endif