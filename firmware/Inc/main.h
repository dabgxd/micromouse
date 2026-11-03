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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define TOF_L90_XSHUT_Pin GPIO_PIN_1
#define TOF_L90_XSHUT_GPIO_Port GPIOA
#define RXD_HC05_Pin GPIO_PIN_2
#define RXD_HC05_GPIO_Port GPIOA
#define TXD_HC05_Pin GPIO_PIN_3
#define TXD_HC05_GPIO_Port GPIOA
#define TOF_L45_XSHUT_Pin GPIO_PIN_4
#define TOF_L45_XSHUT_GPIO_Port GPIOA
#define TOF_M0_XSHUT_Pin GPIO_PIN_5
#define TOF_M0_XSHUT_GPIO_Port GPIOA
#define ENCODER_L_B_Pin GPIO_PIN_6
#define ENCODER_L_B_GPIO_Port GPIOA
#define ENCODER_L_A_Pin GPIO_PIN_7
#define ENCODER_L_A_GPIO_Port GPIOA
#define TOF_R45_XSHUT_Pin GPIO_PIN_0
#define TOF_R45_XSHUT_GPIO_Port GPIOB
#define TOF_R90_XSHUT_Pin GPIO_PIN_1
#define TOF_R90_XSHUT_GPIO_Port GPIOB
#define SCL_MPU6050_Pin GPIO_PIN_10
#define SCL_MPU6050_GPIO_Port GPIOB
#define AIN1_TB6612_Pin GPIO_PIN_12
#define AIN1_TB6612_GPIO_Port GPIOB
#define AIN2_TB6612_Pin GPIO_PIN_13
#define AIN2_TB6612_GPIO_Port GPIOB
#define BIN1_TB6612_Pin GPIO_PIN_14
#define BIN1_TB6612_GPIO_Port GPIOB
#define BIN2_TB6612_Pin GPIO_PIN_15
#define BIN2_TB6612_GPIO_Port GPIOB
#define PWMA_R_TB6612_Pin GPIO_PIN_8
#define PWMA_R_TB6612_GPIO_Port GPIOA
#define PWMB_L_TB6612_Pin GPIO_PIN_9
#define PWMB_L_TB6612_GPIO_Port GPIOA
#define STBY_TB6612_Pin GPIO_PIN_10
#define STBY_TB6612_GPIO_Port GPIOA
#define SDA_MPU6050_Pin GPIO_PIN_3
#define SDA_MPU6050_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_5
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_R_B_Pin GPIO_PIN_6
#define ENCODER_R_B_GPIO_Port GPIOB
#define ENCODER_R_A_Pin GPIO_PIN_7
#define ENCODER_R_A_GPIO_Port GPIOB
#define SCL_TOF_Pin GPIO_PIN_8
#define SCL_TOF_GPIO_Port GPIOB
#define SDA_TOF_Pin GPIO_PIN_9
#define SDA_TOF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
