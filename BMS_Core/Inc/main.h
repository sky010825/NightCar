/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define VOLTAGE_Pin GPIO_PIN_0
#define VOLTAGE_GPIO_Port GPIOA
#define CURRENT_Pin GPIO_PIN_1
#define CURRENT_GPIO_Port GPIOA
#define Putty_TX_Pin GPIO_PIN_2
#define Putty_TX_GPIO_Port GPIOA
#define Putty_RX_Pin GPIO_PIN_3
#define Putty_RX_GPIO_Port GPIOA
#define Server_TX_Pin GPIO_PIN_10
#define Server_TX_GPIO_Port GPIOC
#define Server_RX_Pin GPIO_PIN_11
#define Server_RX_GPIO_Port GPIOC
#define MP3_TX_Pin GPIO_PIN_12
#define MP3_TX_GPIO_Port GPIOC
#define MP3_RX_Pin GPIO_PIN_2
#define MP3_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
