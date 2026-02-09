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
#include "stm32f0xx_hal.h"

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
#define DHT22_Pin GPIO_PIN_1
#define DHT22_GPIO_Port GPIOA
#define GPIO_IO1_Pin GPIO_PIN_7
#define GPIO_IO1_GPIO_Port GPIOA
#define MODULE_M0_Pin GPIO_PIN_0
#define MODULE_M0_GPIO_Port GPIOB
#define MODULE_M1_Pin GPIO_PIN_1
#define MODULE_M1_GPIO_Port GPIOB
#define MPX_select_2_Pin GPIO_PIN_11
#define MPX_select_2_GPIO_Port GPIOA
#define MPX_select_1_Pin GPIO_PIN_12
#define MPX_select_1_GPIO_Port GPIOA
#define MODULE_AUX_Pin GPIO_PIN_15
#define MODULE_AUX_GPIO_Port GPIOA
#define G_COMM_Pin GPIO_PIN_3
#define G_COMM_GPIO_Port GPIOB
#define R_ERROR_Pin GPIO_PIN_4
#define R_ERROR_GPIO_Port GPIOB
#define G_WRK_Pin GPIO_PIN_5
#define G_WRK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
uint32_t encode_message(uint8_t dev_id, uint8_t type, uint16_t value) ;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
