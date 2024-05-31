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
enum RUN_MODES {
  FLIGHT, // "safer" mode; use when propellers are in
  TEST // "unsafe" mode; use when testing code without propellers
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_ADC_Pin GPIO_PIN_13
#define BATT_ADC_GPIO_Port GPIOC
#define MPU6500_MOSI_Pin GPIO_PIN_1
#define MPU6500_MOSI_GPIO_Port GPIOC
#define MPU6500_MISO_Pin GPIO_PIN_2
#define MPU6500_MISO_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define NRF24_MISO_Pin GPIO_PIN_6
#define NRF24_MISO_GPIO_Port GPIOA
#define NRF24_MOSI_Pin GPIO_PIN_7
#define NRF24_MOSI_GPIO_Port GPIOA
#define CAM_SD_MOSI_Pin GPIO_PIN_0
#define CAM_SD_MOSI_GPIO_Port GPIOB
#define MPU6500_SCK_Pin GPIO_PIN_10
#define MPU6500_SCK_GPIO_Port GPIOB
#define MPU6500_CS_Pin GPIO_PIN_12
#define MPU6500_CS_GPIO_Port GPIOB
#define NRF24_CE_Pin GPIO_PIN_7
#define NRF24_CE_GPIO_Port GPIOC
#define FRONT_RIGHT_MOTOR_PWM_Pin GPIO_PIN_8
#define FRONT_RIGHT_MOTOR_PWM_GPIO_Port GPIOA
#define FRONT_LEFT_MOTOR_PWM_Pin GPIO_PIN_9
#define FRONT_LEFT_MOTOR_PWM_GPIO_Port GPIOA
#define REAR_RIGHT_MOTOR_PWM_Pin GPIO_PIN_10
#define REAR_RIGHT_MOTOR_PWM_GPIO_Port GPIOA
#define REAR_LEFT_MOTOR_PWM_Pin GPIO_PIN_11
#define REAR_LEFT_MOTOR_PWM_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA
#define CAM_SD_SCK_Pin GPIO_PIN_10
#define CAM_SD_SCK_GPIO_Port GPIOC
#define CAM_SD_MISO_Pin GPIO_PIN_11
#define CAM_SD_MISO_GPIO_Port GPIOC
#define CAM_CS_Pin GPIO_PIN_12
#define CAM_CS_GPIO_Port GPIOC
#define NRF24_SCK_Pin GPIO_PIN_3
#define NRF24_SCK_GPIO_Port GPIOB
#define NRF24_CS_Pin GPIO_PIN_6
#define NRF24_CS_GPIO_Port GPIOB
#define BMP390_SCL_Pin GPIO_PIN_8
#define BMP390_SCL_GPIO_Port GPIOB
#define BMP390_SDA_Pin GPIO_PIN_9
#define BMP390_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
