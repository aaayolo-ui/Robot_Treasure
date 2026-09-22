/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_A_IN1_Pin GPIO_PIN_0
#define MOTOR_A_IN1_GPIO_Port GPIOC
#define MOTOR_A_IN2_Pin GPIO_PIN_1
#define MOTOR_A_IN2_GPIO_Port GPIOC
#define MOTOR_STBY_Pin GPIO_PIN_2
#define MOTOR_STBY_GPIO_Port GPIOC
#define MOTOR_B_IN1_Pin GPIO_PIN_3
#define MOTOR_B_IN1_GPIO_Port GPIOC
#define WK_UP_Pin GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA
#define MOTOR_B_IN2_Pin GPIO_PIN_4
#define MOTOR_B_IN2_GPIO_Port GPIOC
#define MOTOR_C_IN1_Pin GPIO_PIN_5
#define MOTOR_C_IN1_GPIO_Port GPIOC
#define MOTOR_C_IN2_Pin GPIO_PIN_0
#define MOTOR_C_IN2_GPIO_Port GPIOB
#define MOTOR_D_IN1_Pin GPIO_PIN_1
#define MOTOR_D_IN1_GPIO_Port GPIOB
#define TFT_BLK_Pin GPIO_PIN_4
#define TFT_BLK_GPIO_Port GPIOB
#define TFT_RS_DC_Pin GPIO_PIN_5
#define TFT_RS_DC_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_6
#define TFT_RST_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_7
#define TFT_CS_GPIO_Port GPIOB
#define TFT_SCLK_Pin GPIO_PIN_8
#define TFT_SCLK_GPIO_Port GPIOB
#define TFT_MOSI_Pin GPIO_PIN_9
#define TFT_MOSI_GPIO_Port GPIOB
#define NCHD12_FRONT_SCL_Pin GPIO_PIN_4
#define NCHD12_FRONT_SCL_GPIO_Port GPIOA
#define NCHD12_FRONT_SDA_Pin GPIO_PIN_5
#define NCHD12_FRONT_SDA_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_8
#define KEY2_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_9
#define KEY1_GPIO_Port GPIOC
#define MOTOR_D_IN2_Pin GPIO_PIN_10
#define MOTOR_D_IN2_GPIO_Port GPIOA
#define ENCODER_B_A_Pin GPIO_PIN_10
#define ENCODER_B_A_GPIO_Port GPIOC
#define ENCODER_B_B_Pin GPIO_PIN_11
#define ENCODER_B_B_GPIO_Port GPIOC
#define NCHD12_REAR_SCL_Pin GPIO_PIN_12
#define NCHD12_REAR_SCL_GPIO_Port GPIOB
#define NCHD12_REAR_SDA_Pin GPIO_PIN_13
#define NCHD12_REAR_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
