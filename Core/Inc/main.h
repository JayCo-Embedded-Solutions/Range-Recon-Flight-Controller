/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define MPU6500_MOSI_Pin GPIO_PIN_1
#define MPU6500_MOSI_GPIO_Port GPIOC
#define MPU6500_MISO_Pin GPIO_PIN_2
#define MPU6500_MISO_GPIO_Port GPIOC
#define NRF24_SCK_Pin GPIO_PIN_5
#define NRF24_SCK_GPIO_Port GPIOA
#define NRF24_MISO_Pin GPIO_PIN_6
#define NRF24_MISO_GPIO_Port GPIOA
#define NRF24_MOSI_Pin GPIO_PIN_7
#define NRF24_MOSI_GPIO_Port GPIOA
#define MPU6500_SCK_Pin GPIO_PIN_10
#define MPU6500_SCK_GPIO_Port GPIOB
#define MPU6500_CS_Pin GPIO_PIN_12
#define MPU6500_CS_GPIO_Port GPIOB
#define NRF24_CE_Pin GPIO_PIN_7
#define NRF24_CE_GPIO_Port GPIOC
#define NRF24_CS_Pin GPIO_PIN_6
#define NRF24_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
