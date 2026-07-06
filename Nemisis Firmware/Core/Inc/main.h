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
#include "stm32g4xx_hal.h"

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
#define LEFT_FRONT_EMMITER_Pin GPIO_PIN_13
#define LEFT_FRONT_EMMITER_GPIO_Port GPIOC
#define LEFT_M_EMMITER_Pin GPIO_PIN_14
#define LEFT_M_EMMITER_GPIO_Port GPIOC
#define LEFT_LM_EMMITER_Pin GPIO_PIN_15
#define LEFT_LM_EMMITER_GPIO_Port GPIOC
#define ENCODERL_A_Pin GPIO_PIN_0
#define ENCODERL_A_GPIO_Port GPIOC
#define RIGHT_FRONT_RECEIVER_Pin GPIO_PIN_1
#define RIGHT_FRONT_RECEIVER_GPIO_Port GPIOC
#define LEFT_FRONT_RECEIVER_Pin GPIO_PIN_2
#define LEFT_FRONT_RECEIVER_GPIO_Port GPIOC
#define LEFT_M_RECEIVER_Pin GPIO_PIN_3
#define LEFT_M_RECEIVER_GPIO_Port GPIOC
#define LEFT_LM_RECEIVER_Pin GPIO_PIN_0
#define LEFT_LM_RECEIVER_GPIO_Port GPIOA
#define ENCODERR_A_Pin GPIO_PIN_1
#define ENCODERR_A_GPIO_Port GPIOA
#define RGB_LED_Pin GPIO_PIN_2
#define RGB_LED_GPIO_Port GPIOA
#define RIGHT_M_RECEIVER_Pin GPIO_PIN_3
#define RIGHT_M_RECEIVER_GPIO_Port GPIOA
#define ENCODERL_CS_Pin GPIO_PIN_4
#define ENCODERL_CS_GPIO_Port GPIOA
#define ENCODERR_B_Pin GPIO_PIN_5
#define ENCODERR_B_GPIO_Port GPIOA
#define ENCODERR_CS_Pin GPIO_PIN_2
#define ENCODERR_CS_GPIO_Port GPIOB
#define RIGHT_RM_RECEIVER_Pin GPIO_PIN_12
#define RIGHT_RM_RECEIVER_GPIO_Port GPIOB
#define RIGHT_RM_EMMITER_Pin GPIO_PIN_13
#define RIGHT_RM_EMMITER_GPIO_Port GPIOB
#define M2_CURRENT_SENSE_Pin GPIO_PIN_14
#define M2_CURRENT_SENSE_GPIO_Port GPIOB
#define M1_CURRENT_SENSE_Pin GPIO_PIN_15
#define M1_CURRENT_SENSE_GPIO_Port GPIOB
#define M2_IN1_Pin GPIO_PIN_6
#define M2_IN1_GPIO_Port GPIOC
#define M1_IN2_Pin GPIO_PIN_7
#define M1_IN2_GPIO_Port GPIOC
#define M1_IN1_Pin GPIO_PIN_8
#define M1_IN1_GPIO_Port GPIOC
#define FUEL_SDA_Pin GPIO_PIN_9
#define FUEL_SDA_GPIO_Port GPIOC
#define FUEL_SCL_Pin GPIO_PIN_8
#define FUEL_SCL_GPIO_Port GPIOA
#define ENCODERL_B_Pin GPIO_PIN_9
#define ENCODERL_B_GPIO_Port GPIOA
#define RIGHT_M_EMMITER_Pin GPIO_PIN_10
#define RIGHT_M_EMMITER_GPIO_Port GPIOA
#define ENCODER_SCK_Pin GPIO_PIN_10
#define ENCODER_SCK_GPIO_Port GPIOC
#define ENCODER_MISO_Pin GPIO_PIN_11
#define ENCODER_MISO_GPIO_Port GPIOC
#define ENCODER_MOSI_Pin GPIO_PIN_12
#define ENCODER_MOSI_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOD
#define RIGHT_FRONT_EMMITER_Pin GPIO_PIN_6
#define RIGHT_FRONT_EMMITER_GPIO_Port GPIOB
#define M2_IN2_Pin GPIO_PIN_7
#define M2_IN2_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOB
#define VACCUM_FAN_Pin GPIO_PIN_9
#define VACCUM_FAN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
