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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M3_Step_Pin GPIO_PIN_2
#define M3_Step_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOE
#define M4_Step_Pin GPIO_PIN_4
#define M4_Step_GPIO_Port GPIOE
#define M5_Step_Pin GPIO_PIN_5
#define M5_Step_GPIO_Port GPIOE
#define M6_Step_Pin GPIO_PIN_6
#define M6_Step_GPIO_Port GPIOE
#define M1_EN_Pin GPIO_PIN_0
#define M1_EN_GPIO_Port GPIOC
#define M2_EN_Pin GPIO_PIN_1
#define M2_EN_GPIO_Port GPIOC
#define M3_EN_Pin GPIO_PIN_2
#define M3_EN_GPIO_Port GPIOC
#define M4_EN_Pin GPIO_PIN_3
#define M4_EN_GPIO_Port GPIOC
#define Gripper_LED_PWM_Pin GPIO_PIN_0
#define Gripper_LED_PWM_GPIO_Port GPIOA
#define Buzz_pin_Pin GPIO_PIN_1
#define Buzz_pin_GPIO_Port GPIOA
#define M5_EN_Pin GPIO_PIN_4
#define M5_EN_GPIO_Port GPIOC
#define M6_EN_Pin GPIO_PIN_5
#define M6_EN_GPIO_Port GPIOC
#define M1_ENC_CS_Pin GPIO_PIN_0
#define M1_ENC_CS_GPIO_Port GPIOB
#define M2_ENC_CS_Pin GPIO_PIN_1
#define M2_ENC_CS_GPIO_Port GPIOB
#define M3_ENC_CS_Pin GPIO_PIN_2
#define M3_ENC_CS_GPIO_Port GPIOB
#define Servo1_Pin_Pin GPIO_PIN_9
#define Servo1_Pin_GPIO_Port GPIOE
#define Servo2_Pin_Pin GPIO_PIN_11
#define Servo2_Pin_GPIO_Port GPIOE
#define Servo3_Pin_Pin GPIO_PIN_13
#define Servo3_Pin_GPIO_Port GPIOE
#define RPI_TX_Pin GPIO_PIN_10
#define RPI_TX_GPIO_Port GPIOB
#define RPI_RX_Pin GPIO_PIN_11
#define RPI_RX_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_12
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_14
#define SW3_GPIO_Port GPIOB
#define M2_MS2_Pin GPIO_PIN_8
#define M2_MS2_GPIO_Port GPIOD
#define M3_MS1_Pin GPIO_PIN_9
#define M3_MS1_GPIO_Port GPIOD
#define M3_Dir_Pin GPIO_PIN_10
#define M3_Dir_GPIO_Port GPIOD
#define M6_ENC_CS_Pin GPIO_PIN_15
#define M6_ENC_CS_GPIO_Port GPIOD
#define M5_ENC_CS_Pin GPIO_PIN_6
#define M5_ENC_CS_GPIO_Port GPIOC
#define M4_ENC_CS_Pin GPIO_PIN_7
#define M4_ENC_CS_GPIO_Port GPIOC
#define M1_Dir_Pin GPIO_PIN_0
#define M1_Dir_GPIO_Port GPIOD
#define M2_Dir_Pin GPIO_PIN_1
#define M2_Dir_GPIO_Port GPIOD
#define M5_Dir_Pin GPIO_PIN_4
#define M5_Dir_GPIO_Port GPIOD
#define M6_Dir_Pin GPIO_PIN_5
#define M6_Dir_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_5
#define SPI3_CS_GPIO_Port GPIOB
#define RGB_DIN_Pin GPIO_PIN_6
#define RGB_DIN_GPIO_Port GPIOB
#define M1_Step_Pin GPIO_PIN_7
#define M1_Step_GPIO_Port GPIOB
#define M4_Dir_Pin GPIO_PIN_8
#define M4_Dir_GPIO_Port GPIOB
#define M2_MS1_Pin GPIO_PIN_9
#define M2_MS1_GPIO_Port GPIOB
#define M2_Step_Pin GPIO_PIN_1
#define M2_Step_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
