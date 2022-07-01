/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Retarget.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SPI_TIMEOUT_SECS 	3
#define ONE_BYTE 		1
#define TWO_BYTES		2
#define OK 0
#define UART_TIMEOUT		10
#define I2C_TIMEOUT		10
#define SHT31_ADDRESS 		0x8A // Note: Addr << 1
#define SHORT_BLINK   		100
#define LONG_BLINK 		500
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
enum  rgb_t { RED, YELLOW, GREEN, CYAN, BLUE, MAGENTA, WHITE, BLACK } ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void Error_Msg_Handler(const char *msg);
void EnterStopMode(void);
uint8_t readRegSpi(SPI_HandleTypeDef *hspi, uint8_t reg);
void writeRegSpi(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data);
void setRgbLed(int led);
void spiToLowPowerState(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  // Redirect printf to UART1
  RetargetInit(&huart1);

  // Run at Range 2 (Core voltage 1.5V) @ 4MHz to reduce run mode power slightly
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Check SHT31 communication over I2C1
   uint8_t reset_cmd[2] = {0x30, 0xa2};
   if (HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDRESS, reset_cmd, 2, 1000) == HAL_OK) {
 	  printf("\n\rSHT31 successfully reset");
   }
   else {
 	  Error_Msg_Handler("\n\rSHT31 reset failed!");
   }


   // SX1272 is in FSO/OOK mode after reset
   HAL_GPIO_WritePin (RESET_RF_GPIO_Port, RESET_RF_Pin, GPIO_PIN_RESET);
   HAL_Delay(1);
   HAL_GPIO_WritePin (RESET_RF_GPIO_Port, RESET_RF_Pin, GPIO_PIN_SET);
   HAL_Delay(1);
   HAL_GPIO_WritePin (RESET_RF_GPIO_Port, RESET_RF_Pin, GPIO_PIN_RESET);
   HAL_Delay(10);

   // Check SX1272 SPI2 communication
   uint8_t sx_id = readRegSpi(&hspi2,0x42);
   if (sx_id == 0x22) {
       printf("\n\rSX1272 chip ID read OK: %x", sx_id);
   }
   else {
       Error_Msg_Handler("\n\rSX1272 ID read failure!");
   }

   // Send commands to put Semtech SX1272 to sleep
   // (Don't forget to set Bit7 (MSB) of register address for writing!)
   writeRegSpi(&hspi2, 0xC0, 0xFF); // DIOmapping1  to --
   writeRegSpi(&hspi2, 0xC1, 0xFF); // DIOmapping2 to --
   HAL_Delay(10);
   writeRegSpi(&hspi2, 0x81, 0x01); // CRTL1 to STBY mode
   HAL_Delay(10);
   writeRegSpi(&hspi2, 0xA4, 0x07); // REGOSC to CLKOUT=off
   HAL_Delay(10);
   writeRegSpi(&hspi2, 0x81, 0x00); // CTRL1 to SLEEP mode

   //Change SX1272 to LoRa mode from default FSK/OOK:
   HAL_Delay(10);
   writeRegSpi(&hspi2, 0x81, 0x80); // CTRL1 to SLEEP mode
   HAL_GPIO_WritePin (CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);
   printf("\n\rSX1272 should now be in STANDBY mode...");

   // Read MX25 flash ID, then put flash to deep sleep
  uint8_t read_id_cmd[4] = {0x9F,0x55,0xAA,0x55};
  uint8_t read_result[4];
  // CHeck MX25 SPI1 can read chip ID bytes 0xC22813
  HAL_GPIO_WritePin (FS_CS_GPIO_Port, FS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, read_id_cmd, read_result, 4, SPI_TIMEOUT_SECS);
  HAL_GPIO_WritePin (FS_CS_GPIO_Port, FS_CS_Pin, GPIO_PIN_SET);
  if  (read_result[1]==0xC2 &&  read_result[2]==0x28 && read_result[3]==0x13) {
      printf("\n\rMX25 chip ID read OK: %x %x %x",read_result[1], read_result[2], read_result[3]);
  }
  else {
      Error_Msg_Handler ("MX25 SPI1 ID read failure!");
  }

  uint8_t sleep_command[1] = {0xB9};
  HAL_GPIO_WritePin (FS_CS_GPIO_Port, FS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, sleep_command, 1, SPI_TIMEOUT_SECS);
  HAL_GPIO_WritePin (FS_CS_GPIO_Port, FS_CS_Pin, GPIO_PIN_SET);
  printf("\n\rMX25 put to deep sleep.");



  printf("\n\rAbout to put STM32 to STOP mode...");
  // Blink LED a few times
  for (int i=0; i<3; i++)
  {
	setRgbLed(MAGENTA);
	HAL_Delay(SHORT_BLINK);
	setRgbLed(BLACK);
	HAL_Delay(SHORT_BLINK);
  }

  EnterStopMode();

  for (int i=0; i<3; i++)
  {
	setRgbLed(YELLOW);
	HAL_Delay(SHORT_BLINK);
	setRgbLed(BLACK);
	HAL_Delay(SHORT_BLINK);
  }
  printf("\n\rSTM32 has resumed execution...");

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDF_Pin|FS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LEDR_Pin|LEDG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEDB_Pin|RESET_RF_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13_Pin PC11_Pin PC10_Pin PC12_Pin
                           PC9_Pin PC0_Pin PC7_Pin PC8_Pin
                           PC6_Pin */
  GPIO_InitStruct.Pin = PC13_Pin|PC11_Pin|PC10_Pin|PC12_Pin
                          |PC9_Pin|PC0_Pin|PC7_Pin|PC8_Pin
                          |PC6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA15_Pin PA12_Pin PA11_Pin PA8_Pin
                           PA2_Pin PA5_Pin PA3_Pin */
  GPIO_InitStruct.Pin = PA15_Pin|PA12_Pin|PA11_Pin|PA8_Pin
                          |PA2_Pin|PA5_Pin|PA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDF_Pin */
  GPIO_InitStruct.Pin = LEDF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2_Pin */
  GPIO_InitStruct.Pin = PD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDR_Pin LEDG_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin|LEDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7_Pin PB0_Pin PB1_Pin PB2_Pin */
  GPIO_InitStruct.Pin = PB7_Pin|PB0_Pin|PB1_Pin|PB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDB_Pin RESET_RF_Pin */
  GPIO_InitStruct.Pin = LEDB_Pin|RESET_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin DIO0_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO0_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FS_CS_Pin */
  GPIO_InitStruct.Pin = FS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_RF_Pin */
  GPIO_InitStruct.Pin = CS_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_RF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO3_Pin DIO4_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin|DIO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO5_Pin RF_SW_Pin */
  GPIO_InitStruct.Pin = DIO5_Pin|RF_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void EnterStopMode(void)
{
  // Configure all GPIO port pins to optimal states for low power
  printf("\n\rsetting SPI1/2 lines to low power state...");
  spiToLowPowerState();
  HAL_Delay(5);

  // Disable GPIOs clock
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();


  // Turn off peripherals except RTC
  HAL_ADC_Stop_IT(&hadc);
  HAL_ADC_DeInit(&hadc);
  HAL_SPI_DeInit(&hspi1);
  HAL_SPI_DeInit(&hspi2);
  HAL_I2C_DeInit(&hi2c1);
  HAL_UART_DeInit(&huart1);


  // Clear old RTC Wake Timer settings
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  // Enable wake-up pin
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

  // Set RTC to wake up afer about 28s
   if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xFFFF, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
       Error_Msg_Handler("HAL_RTCEx_SetWakeUpTimer_IT failure!");
  }

  // Stop the 1ms SysTick
  HAL_SuspendTick();

  // Disable the Fast WakeUp from Ultra Low Power mode
  HAL_PWREx_DisableFastWakeUp();

  // set ULP bit for further power saving, see TRM p154
  HAL_PWREx_EnableUltraLowPower();

  // Clear the WU FLAG
   __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  // Clear the RTC Wake UP (WU) flag
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  // Make sure event register is cleared before enabling interrupts or entering stop mode
  __SEV(); // Manally set an event
  __WFE(); // Then wait for event. This will then immediately clear the event register

  // Enable interrupt servicing
  __enable_irq();

  // Enter Stop Mode
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  //HAL_PWR_EnterSTANDBYMode();

  //....WAKE UP CONTINUES FROM HERE...

  // init the SYS Clocks
  SystemClock_Config();
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_ResumeTick();


  /* Clear Peripheral States for MspInit */
  huart1.gState = HAL_UART_STATE_RESET;
  hi2c1.State   = HAL_I2C_STATE_RESET;
  hadc.State    = HAL_ADC_STATE_RESET;
  hspi1.State    = HAL_SPI_STATE_RESET;
  hspi2.State    = HAL_SPI_STATE_RESET;


  /* Re-initialize things */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_ADC_Init();
}

void writeRegSpi(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data)
{
  HAL_GPIO_WritePin (CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_RESET);
  uint8_t reg_data[2] = {reg, data};
  HAL_SPI_Transmit(hspi, reg_data, TWO_BYTES, SPI_TIMEOUT_SECS);
HAL_GPIO_WritePin (CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);

}

uint8_t readRegSpi(SPI_HandleTypeDef *hspi, uint8_t reg)
{
  uint8_t data = 0;
  HAL_GPIO_WritePin (CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &reg, ONE_BYTE, SPI_TIMEOUT_SECS);
  HAL_SPI_Receive(hspi,  &data, ONE_BYTE, SPI_TIMEOUT_SECS);
  HAL_GPIO_WritePin (CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);
return data;
}


void setRgbLed(int led) {

  switch (led)
  {
  case RED:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	    break;
  case YELLOW:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	    break;
  case GREEN:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	    break;
  case CYAN:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
		    break;
  case BLUE:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	    break;
  case MAGENTA:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	    break;
  case WHITE:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	    break;
  case BLACK:
	    HAL_GPIO_WritePin (LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin (LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	    break;

  }
}

void spiToLowPowerState(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // SPI1 to MX25 Flash - drive lines as low outputs
  HAL_SPI_DeInit(&hspi1);
  GPIO_InitStruct.Pin = SCK_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin (SCK_P_GPIO_Port, SCK_P_Pin, GPIO_PIN_RESET);
  HAL_GPIO_Init(SCK_P_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOSI_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin (MOSI_P_GPIO_Port, MOSI_P_Pin, GPIO_PIN_RESET);
  HAL_GPIO_Init(MOSI_P_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MISO_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin (MISO_P_GPIO_Port, MISO_P_Pin, GPIO_PIN_RESET);
  HAL_GPIO_Init(MISO_P_GPIO_Port, &GPIO_InitStruct);

  // SPI2 to SX1272 - drive lines as low outputs
  HAL_SPI_DeInit(&hspi2);
  GPIO_InitStruct.Pin = SCK_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK_RF_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOSI_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOSI_RF_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MISO_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MISO_RF_GPIO_Port, &GPIO_InitStruct);
}

void Error_Msg_Handler(const char *msg)
{
  // My version of Error Handler that prints a message
  __disable_irq();
  printf(msg);
  while (1)
  {
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
