/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc;

/* USER CODE BEGIN Private defines */
#define EXTERNAL_ADC 0
#define EXTERNAL_BOTTOM_RESISTOR 5000

#define CURRENT_ADC 1
#define CURRENT_SENSE_RESISTOR 0.082
#define CURRENT_SENSE_GAIN 50

#define VOLTAGE_ADC 2
#define VOLTAGE_TOP_RESISTOR 20000
#define VOLTAGE_BOTTOM_RESISTOR 5000

#define ADC_MAX 4095
#define ADC_REF 3.3
/* USER CODE END Private defines */

void MX_ADC_Init(void);

/* USER CODE BEGIN Prototypes */
float getVoltage();
float getCurrent();
float getExternalVoltage();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

