/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shared.h"
#include "dac.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* data */
  uint8_t               Channelone_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
uint8_t SAMPLETIME[8]={ADC_SAMPLETIME_2CYCLES_5,
		  ADC_SAMPLETIME_6CYCLES_5,
		  ADC_SAMPLETIME_12CYCLES_5,
		  ADC_SAMPLETIME_24CYCLES_5,
		  ADC_SAMPLETIME_47CYCLES_5,
		  ADC_SAMPLETIME_92CYCLES_5,
		  ADC_SAMPLETIME_247CYCLES_5,
		  ADC_SAMPLETIME_640CYCLES_5};
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Prototype

//External buffer
uint8_t ble_flag = 1;
uint8_t ble_buffer[1]={0};

extern uint16_t *myPointerToDMA = NULL;
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* data */
static void Custom_Channelone_Update_Char(void);
static void Custom_Channelone_Send_Notification(void);

/* USER CODE BEGIN PFP */
extern void newConfigReceived(){
	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	if(*myPointerToConfigArray ==0x01){
		startADC();
	}else{
		//TODO
	}
}

extern void dac_config_received(){
	UTIL_SEQ_SetTask(1 << CFG_TASK_UPDATE_DAC, CFG_SCH_PRIO_0);
}
extern void adc_config_received(){
	UTIL_SEQ_SetTask(1 << CFG_TASK_UPDATE_ADC, CFG_SCH_PRIO_0);
}
void update_adc_settings(void){


	uint8_t adc_enable;
	uint8_t adc_mode;
	uint8_t adc_sampletime;
	uint8_t adc_clock_prescaler;
	uint8_t adc_oversampling;

	//adc_char_length


	if(p_adc_config != NULL && *p_adc_config == 1){

		adc_enable = 1;//p_adc_config;
		adc_mode = 1;//p_adc_config+1;
		adc_sampletime = 7;//p_adc_config+2;
		adc_clock_prescaler = ADC_CLOCK_ASYNC_DIV32;//p_adc_config+4;
		adc_oversampling = 1;//p_adc_config+5;
		//if(adc_mode == 1){
		if(1){
			//live mode; channel 1;

			HAL_ADC_Stop_DMA(&hadc1);
			hadc1.Instance = ADC1;
			hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
			hadc1.Init.Resolution = ADC_RESOLUTION_12B;
			hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
			hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
			hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
			hadc1.Init.LowPowerAutoWait = DISABLE;
			hadc1.Init.ContinuousConvMode = ENABLE;
			hadc1.Init.NbrOfConversion = 1;
			hadc1.Init.DiscontinuousConvMode = DISABLE;
			hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
			hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
			hadc1.Init.DMAContinuousRequests = ENABLE;
			hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
			hadc1.Init.OversamplingMode = ENABLE;
			hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;//ADC_OVERSAMPLING_RATIO_64;
			hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
			hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
			hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;


			if (HAL_ADC_Init(&hadc1) != HAL_OK)
			{
			Error_Handler();
			}
			ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_14;//ADC_CHANNEL_14; oder_2
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;//SAMPLETIME[adc_sampletime];
			sConfig.SingleDiff = ADC_SINGLE_ENDED;
			sConfig.OffsetNumber = ADC_OFFSET_NONE;
			sConfig.Offset = 0;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
			Error_Handler();
			}
			startADC();
			//HAL_GetTick();
		}
		adc_char_length = 180;
		startADC();
	}

}

void update_dac_settings(void){

	uint8_t* dac_enable;
	uint8_t* dac_mode;
	uint8_t* dac_value;
	if(p_dac_config != NULL){
		dac_enable = p_dac_config;
		dac_mode = p_dac_config +1;
		dac_value = p_dac_config +2;

		if(*dac_enable == 1 && *dac_mode == 1){
			float target[1];
			memcpy(&target[0],dac_value,4);
			float my_dac_val = (9.9-target[0])*2.9/(9.9-(-8.18));
			dacx3202_set_voltage(&dacx3202, DACX3202_DAC_0, my_dac_val);
		}
	}
}
void myTask(void){
	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+adc_char_length/2);
	}else{
		//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)UpdateCharData);
	}
	transferring = 0;
}
void callback_half_filled(void){
	//HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA);
	}
}
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* data */
    case CUSTOM_STM_CHANNELONE_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHANNELONE_READ_EVT */

      /* USER CODE END CUSTOM_STM_CHANNELONE_READ_EVT */
      break;

    case CUSTOM_STM_CHANNELONE_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHANNELONE_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_CHANNELONE_WRITE_EVT */
      break;

    case CUSTOM_STM_CHANNELONE_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHANNELONE_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_CHANNELONE_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CHANNELONE_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHANNELONE_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_CHANNELONE_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_MYCHARNOTIFY_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MYCHARNOTIFY_READ_EVT */

      /* USER CODE END CUSTOM_STM_MYCHARNOTIFY_READ_EVT */
      break;

    case CUSTOM_STM_MYCHARNOTIFY_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MYCHARNOTIFY_WRITE_EVT */

    		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

      /* USER CODE END CUSTOM_STM_MYCHARNOTIFY_WRITE_EVT */
      break;

    case CUSTOM_STM_DAC_CHAR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DAC_CHAR_READ_EVT */

      /* USER CODE END CUSTOM_STM_DAC_CHAR_READ_EVT */
      break;

    case CUSTOM_STM_DAC_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DAC_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_DAC_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_ADC_CFG_CHAR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ADC_CFG_CHAR_READ_EVT */

      /* USER CODE END CUSTOM_STM_ADC_CFG_CHAR_READ_EVT */
      break;

    case CUSTOM_STM_ADC_CFG_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ADC_CFG_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_ADC_CFG_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	UpdateCharData[0]=2;
	Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)UpdateCharData);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

void ble_updateChar(uint16_t* dma_adc)
{

		memcpy(&UpdateCharData[0],dma_adc,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)UpdateCharData);
		//ble_buffer[0]=1;
		//memcpy(&UpdateCharData[0],&ble_buffer[0],1);
		//Custom_Datac_Update_Char();
		//uint8_t testval[1];
		//testval[0]=1;
		//Custom_STM_App_Update_Char(CUSTOM_STM_DATAC, testval);
		//Custom_STM_App_Update_Char(CUSTOM_STM_DATAC, (uint8_t *)testval);
		//Custom_Datac_Send_Notification();
		//Custom_Datac_Update_Char();

		//Custom_Datac_Update_Char();
		//Custom_Datac_Send_Notification();


	//UpdateCharData[0] ^= 0x1;
	//HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
	//Custom_Datac_Update_Char();
	//Custom_Datac_Send_Notification();
		//UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);
		return;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* data */
void Custom_Channelone_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Channelone_UC_1*/

  /* USER CODE END Channelone_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Channelone_UC_Last*/

  /* USER CODE END Channelone_UC_Last*/
  return;
}

void Custom_Channelone_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Channelone_NS_1*/

  /* USER CODE END Channelone_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Channelone_NS_Last*/

  /* USER CODE END Channelone_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
