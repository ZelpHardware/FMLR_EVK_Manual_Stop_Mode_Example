/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define I2C1SDA_Pin GPIO_PIN_9
#define I2C1SDA_GPIO_Port GPIOB
#define MISO_P_Pin GPIO_PIN_4
#define MISO_P_GPIO_Port GPIOB
#define SCK_P_Pin GPIO_PIN_3
#define SCK_P_GPIO_Port GPIOB
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
#define LEDF_Pin GPIO_PIN_8
#define LEDF_GPIO_Port GPIOB
#define PD2_Pin GPIO_PIN_2
#define PD2_GPIO_Port GPIOD
#define PC11_Pin GPIO_PIN_11
#define PC11_GPIO_Port GPIOC
#define PC10_Pin GPIO_PIN_10
#define PC10_GPIO_Port GPIOC
#define PA12_Pin GPIO_PIN_12
#define PA12_GPIO_Port GPIOA
#define LEDR_Pin GPIO_PIN_0
#define LEDR_GPIO_Port GPIOH
#define PB7_Pin GPIO_PIN_7
#define PB7_GPIO_Port GPIOB
#define MOSI_P_Pin GPIO_PIN_5
#define MOSI_P_GPIO_Port GPIOB
#define PC12_Pin GPIO_PIN_12
#define PC12_GPIO_Port GPIOC
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define PA11_Pin GPIO_PIN_11
#define PA11_GPIO_Port GPIOA
#define LEDG_Pin GPIO_PIN_1
#define LEDG_GPIO_Port GPIOH
#define I2C1CLK_Pin GPIO_PIN_6
#define I2C1CLK_GPIO_Port GPIOB
#define PA8_Pin GPIO_PIN_8
#define PA8_GPIO_Port GPIOA
#define PC9_Pin GPIO_PIN_9
#define PC9_GPIO_Port GPIOC
#define LEDB_Pin GPIO_PIN_1
#define LEDB_GPIO_Port GPIOC
#define PC0_Pin GPIO_PIN_0
#define PC0_GPIO_Port GPIOC
#define PC7_Pin GPIO_PIN_7
#define PC7_GPIO_Port GPIOC
#define PC8_Pin GPIO_PIN_8
#define PC8_GPIO_Port GPIOC
#define RESET_RF_Pin GPIO_PIN_2
#define RESET_RF_GPIO_Port GPIOC
#define PA2_Pin GPIO_PIN_2
#define PA2_GPIO_Port GPIOA
#define PA5_Pin GPIO_PIN_5
#define PA5_GPIO_Port GPIOA
#define PB0_Pin GPIO_PIN_0
#define PB0_GPIO_Port GPIOB
#define PC6_Pin GPIO_PIN_6
#define PC6_GPIO_Port GPIOC
#define MOSI_RF_Pin GPIO_PIN_15
#define MOSI_RF_GPIO_Port GPIOB
#define MISO_RF_Pin GPIO_PIN_14
#define MISO_RF_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define PA3_Pin GPIO_PIN_3
#define PA3_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_6
#define DIO1_GPIO_Port GPIOA
#define PB1_Pin GPIO_PIN_1
#define PB1_GPIO_Port GPIOB
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define SCK_RF_Pin GPIO_PIN_10
#define SCK_RF_GPIO_Port GPIOB
#define FS_CS_Pin GPIO_PIN_13
#define FS_CS_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_1
#define DIO0_GPIO_Port GPIOA
#define CS_RF_Pin GPIO_PIN_4
#define CS_RF_GPIO_Port GPIOA
#define DIO2_Pin GPIO_PIN_7
#define DIO2_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_4
#define DIO3_GPIO_Port GPIOC
#define DIO4_Pin GPIO_PIN_5
#define DIO4_GPIO_Port GPIOC
#define DIO5_Pin GPIO_PIN_11
#define DIO5_GPIO_Port GPIOB
#define RF_SW_Pin GPIO_PIN_12
#define RF_SW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
