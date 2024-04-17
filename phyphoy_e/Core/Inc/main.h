/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dac.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint16_t *myPointerToDMA;
extern uint8_t *myPointerToConfigArray;
extern uint8_t *p_dac_config;
extern uint8_t *p_adc_config;
extern uint8_t adc_config[20];
extern volatile uint8_t trasferring;
extern volatile uint8_t activate_trigger;

extern dacx3202_t dacx3202;

extern uint8_t adc_char_length;

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
extern void newConfigReceived();
extern void dac_config_received();
extern void adc_config_received();
extern void setNewADC();
extern void startADC();
extern void set_dma_circular(uint8_t b);

extern void change_edge(uint8_t e);


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern COMP_HandleTypeDef hcomp1;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define in_dac_trigger_Pin GPIO_PIN_2
#define in_dac_trigger_GPIO_Port GPIOA
#define ADC_MUX_divided_Pin GPIO_PIN_1
#define ADC_MUX_divided_GPIO_Port GPIOA
#define DAC_Trigger_COMP1_Pin GPIO_PIN_0
#define DAC_Trigger_COMP1_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define switchRange_Pin GPIO_PIN_5
#define switchRange_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOB
#define ADC_CH1_COMP_P_Pin GPIO_PIN_5
#define ADC_CH1_COMP_P_GPIO_Port GPIOC
#define MUXA1_Pin GPIO_PIN_8
#define MUXA1_GPIO_Port GPIOA
#define MUXA0_Pin GPIO_PIN_9
#define MUXA0_GPIO_Port GPIOA
#define GROUND_Pin GPIO_PIN_4
#define GROUND_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
