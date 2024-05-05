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

	//HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	/*
	if(*myPointerToConfigArray ==0x01){
		//startADC();
		activate_trigger = 1;
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		printf("activate trigger \r\n");
	}else{
		//TODO
	}*/
	start_circular_adc();

}

extern void dac_config_received(){
	UTIL_SEQ_SetTask(1 << CFG_TASK_UPDATE_DAC, CFG_SCH_PRIO_0);
}
extern void adc_config_received(){
	UTIL_SEQ_SetTask(1 << CFG_TASK_UPDATE_ADC, CFG_SCH_PRIO_0);
}

void update_adc_settings(void){

	//adc_char_length

	if(p_adc_config != NULL){

		new_adc_init();
		/*

		change_edge(*adc_edge);
		HAL_ADC_Stop_DMA(&hadc1);
		//HAL_ADC_DeInit(&hadc1);
		hadc1.Instance = ADC1;
		hadc1.Init.ClockPrescaler = PRESCALER[*adc_clock_prescaler];
		hadc1.Init.Resolution = ADC_RESOLUTION_12B;
		hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1.Init.LowPowerAutoWait = DISABLE;
		hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
		hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		hadc1.Init.ContinuousConvMode = ENABLE;
		hadc1.Init.NbrOfConversion = 1;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
		hadc1.Init.DMAContinuousRequests = ENABLE;
		hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
		if(*adc_oversampling==0){
			hadc1.Init.OversamplingMode = DISABLE;
			hadc1.Init.Oversampling.Ratio = OVERSAMPLING[*adc_oversampling];
		}else{
			hadc1.Init.OversamplingMode = ENABLE;
			hadc1.Init.Oversampling.Ratio = OVERSAMPLING[*adc_oversampling];
			hadc1.Init.Oversampling.RightBitShift = BITSHIFT[*adc_oversampling];
			hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
			hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
		}

		if (HAL_ADC_Init(&hadc1) != HAL_OK)
		{
		Error_Handler();
		}
		*/
		/*
		static ADC_ChannelConfTypeDef sConfig = {0};
		if(*adc_routing == 1){
			sConfig.Channel = ADC_CHANNEL_14;
		}else if(*adc_routing == 2){
			sConfig.Channel = ADC_CHANNEL_2;
		}

		if(*adc_mode == 0){
			//live mode;
			set_dma_circular(1);

			//sConfig.Channel = ADC_CHANNEL_14;//ADC_CHANNEL_14; oder_2
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = SAMPLETIME[*adc_sampletime];
			sConfig.SingleDiff = ADC_SINGLE_ENDED;
			sConfig.OffsetNumber = ADC_OFFSET_NONE;
			sConfig.Offset = 0;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
			Error_Handler();
			}
			startADC();
		}

		if(*adc_mode == 1){
			//oscillator-mode
			set_dma_circular(1);

			//sConfig.Channel = ADC_CHANNEL_14;//ADC_CHANNEL_14; oder_2
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = SAMPLETIME[*adc_sampletime];
			sConfig.SingleDiff = ADC_SINGLE_ENDED;
			sConfig.OffsetNumber = ADC_OFFSET_NONE;
			sConfig.Offset = 0;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
			Error_Handler();
			}
			my_prescaler = (12.5 + (uint32_t)(OVERSAMPLING_DIVIDER[*adc_oversampling]*SAMPLETIME_CYCLES[*adc_sampletime]))*PRESCALER_DIVIDER[*adc_clock_prescaler]/(2*10);
			printf("OVERSAMPLING_DIVIDER: %i\r\n",OVERSAMPLING_DIVIDER[*adc_oversampling]);
			printf("SAMPLETIME_CYCLES: %i\r\n",(uint32_t)(OVERSAMPLING_DIVIDER[*adc_oversampling]*SAMPLETIME_CYCLES[*adc_sampletime]));
			printf("PRESCALER_DIVIDER: %i\r\n",PRESCALER_DIVIDER[*adc_clock_prescaler]);
			printf("my_prescaler: %i\r\n",my_prescaler);
			start_circular_adc();
		}
		*/
		//adc_char_length = 180;

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
	/*
	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+adc_char_length/2);
	}
	*/
	printf("lets send data\r\n");
	uint8_t data_buffer[180*5];
	uint8_t* data_buffer_p = &data_buffer[0];
	if(timestamp_trigger >SAMPLES_PRE_TRIGGER && timestamp_trigger+SAMPLES_POST_TRIGGER < ADC_BUFFER_LEN){
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2-SAMPLES_PRE_TRIGGER*2);//send pre trigger 180bytes
		HAL_Delay(10);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2);
		HAL_Delay(10);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2+180);
		HAL_Delay(10);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2+180*2);
		//HAL_Delay(10);
		//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2+180*3);

	}else if(timestamp_trigger < SAMPLES_PRE_TRIGGER){
		printf("send else if \r\n");
		uint16_t s = (ADC_BUFFER_LEN-1)-(SAMPLES_PRE_TRIGGER-timestamp_trigger);
		uint16_t s_length = (SAMPLES_PRE_TRIGGER-timestamp_trigger)*2;
		uint16_t e = timestamp_trigger+SAMPLES_POST_TRIGGER;
		uint16_t e_length = (timestamp_trigger+SAMPLES_POST_TRIGGER)*2;
		/*
		printf("s: %i\r\n",s);
		printf("slength: %i\r\n",s_length);
		printf("e: %i\r\n",e);
		printf("elengh: %i\r\n",e_length);
		*/
		if(s<0 || s>=ADC_BUFFER_LEN){
			return;
		}
		if(e<0 || e>=ADC_BUFFER_LEN){
			return;
		}
		memcpy(&data_buffer[0],(uint8_t *)myPointerToDMA+s*2,s_length);
		memcpy(&data_buffer[0]+s_length,(uint8_t *)myPointerToDMA,e_length);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p);


		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*2);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*3);
		//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*4);


	}else if(timestamp_trigger+ SAMPLES_POST_TRIGGER>ADC_BUFFER_LEN){
		printf("send else if 2\r\n");

		uint16_t s = timestamp_trigger-SAMPLES_PRE_TRIGGER;
		uint16_t s_length = ((ADC_BUFFER_LEN-timestamp_trigger)+SAMPLES_PRE_TRIGGER)*2;
		uint16_t e = timestamp_trigger + SAMPLES_POST_TRIGGER - ADC_BUFFER_LEN;
		uint16_t e_length = e*2;
		/*
		 * printf("s: %i\r\n",s);
		printf("slength: %i\r\n",s_length);
		printf("e: %i\r\n",e);
		printf("elengh: %i\r\n",e_length);
		*/
		if(s<0 || s>=ADC_BUFFER_LEN){
			return;
		}
		if(e<0 || e>=ADC_BUFFER_LEN){
					return;
		}
		memcpy(&data_buffer[0],(uint8_t *)myPointerToDMA+s*2,s_length);
		memcpy(&data_buffer[0]+s_length,(uint8_t *)myPointerToDMA,e_length);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*2);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*3);
		//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*4);
	}

}
void live_first_half(void){
	//HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA);
	}
}
void live_second_half(void){

	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA)+180;
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
