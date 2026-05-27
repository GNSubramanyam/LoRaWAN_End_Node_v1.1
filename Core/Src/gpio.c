/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPDT_VDD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPDT_CNTRL_GPIO_Port, SPDT_CNTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BMM350_INT_Pin */
  GPIO_InitStruct.Pin = BMM350_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMM350_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADAR_CNTRL_Pin RADAR_EN_Pin RADAR_RSTN_Pin RADAR_PWR_Pin
                           MAGNETIC_PWR_Pin SPDT_VDD_Pin */
  GPIO_InitStruct.Pin = RADAR_CNTRL_Pin|RADAR_EN_Pin|RADAR_RSTN_Pin|RADAR_PWR_Pin
                          |MAGNETIC_PWR_Pin|SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDT_VDD_Pin */
  GPIO_InitStruct.Pin = SPDT_VDD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPDT_VDD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADAR_INT_Pin */
  GPIO_InitStruct.Pin = RADAR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADAR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDT_CNTRL_Pin */
  GPIO_InitStruct.Pin = SPDT_CNTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPDT_CNTRL_GPIO_Port, &GPIO_InitStruct);
#if 0
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
