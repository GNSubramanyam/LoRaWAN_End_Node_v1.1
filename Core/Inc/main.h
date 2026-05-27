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
#include "stm32wlxx_hal.h"

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
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define BMM350_INT_Pin GPIO_PIN_8
#define BMM350_INT_GPIO_Port GPIOB
#define BMM350_INT_EXTI_IRQn EXTI9_5_IRQn
#define RADAR_CNTRL_Pin GPIO_PIN_0
#define RADAR_CNTRL_GPIO_Port GPIOA
#define RADAR_EN_Pin GPIO_PIN_1
#define RADAR_EN_GPIO_Port GPIOA
#define RADAR_RSTN_Pin GPIO_PIN_2
#define RADAR_RSTN_GPIO_Port GPIOA
#define RADAR_INT_Pin GPIO_PIN_3
#define RADAR_INT_GPIO_Port GPIOA
#define RADAR_INT_EXTI_IRQn EXTI3_IRQn
#define SPI1_SS_Pin GPIO_PIN_4
#define SPI1_SS_GPIO_Port GPIOA
#define RADAR_PWR_Pin GPIO_PIN_5
#define RADAR_PWR_GPIO_Port GPIOA
#define MAGNETIC_PWR_Pin GPIO_PIN_6
#define MAGNETIC_PWR_GPIO_Port GPIOA
#define SPDT_VDD_Pin GPIO_PIN_8
#define SPDT_VDD_GPIO_Port GPIOA
#define SPDT_CNTRL_Pin GPIO_PIN_12
#define SPDT_CNTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
