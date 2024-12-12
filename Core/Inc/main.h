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
#include "stm32h7xx_hal.h"

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
#define V24_CH1_Pin GPIO_PIN_0
#define V24_CH1_GPIO_Port GPIOF
#define V24_CH2_Pin GPIO_PIN_1
#define V24_CH2_GPIO_Port GPIOF
#define V24_CH3_Pin GPIO_PIN_2
#define V24_CH3_GPIO_Port GPIOF
#define Accel_CS_Pin GPIO_PIN_3
#define Accel_CS_GPIO_Port GPIOF
#define Gyro_CS_Pin GPIO_PIN_4
#define Gyro_CS_GPIO_Port GPIOF
#define Accel_INT_Pin GPIO_PIN_5
#define Accel_INT_GPIO_Port GPIOF
#define Accel_INT_EXTI_IRQn EXTI9_5_IRQn
#define Gyro_INT_Pin GPIO_PIN_8
#define Gyro_INT_GPIO_Port GPIOF
#define Gyro_INT_EXTI_IRQn EXTI9_5_IRQn
#define DBUS_RX_Pin GPIO_PIN_7
#define DBUS_RX_GPIO_Port GPIOE
#define NO_USED_Pin GPIO_PIN_8
#define NO_USED_GPIO_Port GPIOE
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOD
#define WS2812_Pin GPIO_PIN_15
#define WS2812_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
