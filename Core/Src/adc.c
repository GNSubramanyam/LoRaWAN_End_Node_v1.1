/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

ADC_HandleTypeDef hadc;

/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;

  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /* -------- ADD THIS PART -------- */

  sConfig.Channel = ADC_CHANNEL_4;              // PB2
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC)
  {
  /* USER CODE BEGIN ADC_MspInit 0 */

  /* USER CODE END ADC_MspInit 0 */
    /* ADC clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC GPIO Configuration
    PB2     ------> ADC_IN4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_MspInit 1 */

  /* USER CODE END ADC_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC)
  {
  /* USER CODE BEGIN ADC_MspDeInit 0 */

  /* USER CODE END ADC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC GPIO Configuration
    PB2     ------> ADC_IN4
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

  /* USER CODE BEGIN ADC_MspDeInit 1 */

  /* USER CODE END ADC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* Full-scale reference for the battery measurement on ADC_IN4 (PB2), in volts.
 * This is (VREF+ at the ADC) * (battery divider ratio). VERIFY against the
 * board schematic — the previous 1.8 was unconfirmed and may be wrong if
 * VDDA/VREF+ is 3.3 V or if the divider ratio differs. This only sets the
 * scale/threshold accuracy; it does not affect the read-reliability fix below. */
#define ADC_BATT_FULLSCALE_V   1.8f

/**
  * @brief  Read battery voltage on-demand from ADC_IN4 (PB2).
  *         Self-contained: (re)initialises and configures the ADC every call,
  *         then tears it down again. This is required because the LoRaWAN stack
  *         callbacks (SYS_GetBatteryLevel / SYS_GetTemperatureLevel ->
  *         ADC_ReadChannels) HAL_ADC_DeInit() the shared ADC when they finish.
  *         Without re-init, this read used to run on a dead ADC, return 0.0 V,
  *         and trip a false battery-low alert. We now own the full setup/teardown.
  * @retval Battery voltage in volts, or 0.0 only on a genuine conversion failure.
  */
float ADC_ReadBatteryVoltage(void)
{
  float voltage = 0.0f;
  uint32_t rawValue = 0;

  /* Bring the ADC up and configure ADC_CHANNEL_4 (PB2). MX_ADC_Init() both
   * initialises the peripheral and selects channel 4, regardless of whatever
   * state the stack left the ADC in. */
  MX_ADC_Init();

  /* Calibrate ADC for better accuracy */
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    HAL_ADC_DeInit(&hadc);
    return 0.0f;
  }

  /* Start conversion */
  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    /* Wait for conversion to complete (timeout 100 ms) */
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
    {
      rawValue = HAL_ADC_GetValue(&hadc);
      /* Convert 12-bit ADC value to voltage. */
      voltage = ((float)rawValue * ADC_BATT_FULLSCALE_V) / 4095.0f;
    }
    HAL_ADC_Stop(&hadc);
  }

  /* Leave the ADC powered down, matching the state the stack's own ADC users
   * expect between calls. */
  HAL_ADC_DeInit(&hadc);

  return voltage;
}
/* USER CODE END 1 */
