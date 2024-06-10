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
extern void hw_config_received(){
	//git new hw calibration
	printf("hw config: [");
	for(int i=0;i<20;i++){
		printf("%i,",hw_config[i]);
	}
	printf("]\r\n");
	//channel, low/high, int, voltage

	if(hw_config[0]==254){
		SERIALNUMBER[0]=hw_config[1];
		printf("new serialnumber: %i \r\n",SERIALNUMBER[0]);
		return;
	}
	if(hw_config[0]==255){

			printf("store calibration data to flash!\r\n");
			update_flash();
			return;
		}
	if(hw_config[0]==252){
		//calibrate dac
		printf("store calibration data for dac!\r\n");
		uint16_t val_buffer[1]={0};
		memcpy(&val_buffer[0],&hw_config[0]+2,2);
		printf("low/high: %i, measuredValue; %i\r\n",hw_config[1],val_buffer[0]);

		float val_f = (val_buffer[0])*28/4096 -14.0;
		printf("val_f: %f\r\n",val_f);
		CALI_DAC_INT[2+hw_config[1]]= val_buffer[0];
		return;
	}
	if(hw_config[0]==253){
		CALIBRATED=hw_config[1];
		printf("is calibrated \r\n");
		printf("CALI_LOW_INT: %i \r\n",CALI_LOW_INT[0]);
		printf("CALI_HIGH_INT: %i \r\n",CALI_HIGH_INT[0]);

		return;
	}
	uint16_t int_val[1];
	float float_val[1];
	memcpy(&int_val[0],&hw_config[0]+2,2);
	memcpy(&float_val[0],&hw_config[0]+4,4);
	printf("int: %i, float: %f\r\n",int_val[0],float_val[0]);

	if(hw_config[1]==0){
		CALI_LOW_INT[hw_config[0]] = int_val[0];
		CALI_LOW_FLOAT[hw_config[0]] = float_val[0];
	}else if(hw_config[1]==1){
		CALI_HIGH_INT[hw_config[0]] = int_val[0];
		CALI_HIGH_FLOAT[hw_config[0]] = float_val[0];
	}



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
		if(*dac_enable == 254){
			//set dac to 0 for calibration
			dacx3202_set_value(&dacx3202, DACX3202_DAC_0, 1023);
			return;
		}
		if(*dac_enable == 255){
					//set dac to 0 for calibration
					dacx3202_set_value(&dacx3202, DACX3202_DAC_0, 0);
					return;
				}
		/*if(*dac_enable == 1 && *dac_mode == 1){
			float target[1];
			memcpy(&target[0],dac_value,4);
			float my_dac_val = (9.9-target[0])*2.9/(9.9-(-8.18));
			dacx3202_set_voltage(&dacx3202, DACX3202_DAC_0, my_dac_val);
		}
		*/
		if(*dac_enable == 1 && *dac_mode == 1){
			float target[1];
			memcpy(&target[0],dac_value,4);
			printf("target float: %f\r\n",target[0]);
			float m = (CALI_HIGH_INT[0]-CALI_LOW_INT[0])/CALI_HIGH_FLOAT[0];
			float b = CALI_LOW_INT[0];

			float dac_low = (CALI_DAC_INT[2]-b)/m;
			float dac_high = (CALI_DAC_INT[3]-b)/m;

			uint16_t dac_val_calibrated = (target[0]-dac_low)*(1023)/(dac_high-dac_low);
			printf("set dac to val: %i\r\n",dac_val_calibrated);
			dacx3202_set_value(&dacx3202, DACX3202_DAC_0, dac_val_calibrated);
		}


	}
}



uint16_t transfer_to_e_calibration(uint16_t count, uint16_t c){
	//float measure_voltage = (count-CALI_LOW_INT[c])*(CALI_HIGH_FLOAT[c]-CALI_LOW_FLOAT[c])/(CALI_HIGH_INT[c]-CALI_LOW_INT[c])+CALI_LOW_FLOAT[c];

	float m = (CALI_HIGH_INT[c]-CALI_LOW_INT[c])/CALI_HIGH_FLOAT[c];
	float b = CALI_LOW_INT[c];
	float measure_voltage = (count-b)/m;
	//float m = 2048.0/(CALI_HIGH_INT[c]-CALI_LOW_INT[c]);
	//float b = CALI_LOW_INT[c]-2048;
	//float measure_voltage = m*count+b;
	if(measure_voltage > 14){
		measure_voltage = 14.0;
	}else if(measure_voltage < -14){
		measure_voltage = -14;
	}
	return 2048+measure_voltage*(4096/28);
}


void myTask(void){
	/*
	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+adc_char_length/2);
	}
	*/
	printf("lets send data\r\n");
	uint16_t data_buffer[90*5];
	uint8_t* data_buffer_p = &data_buffer[0];
	if(timestamp_trigger >SAMPLES_PRE_TRIGGER && timestamp_trigger+SAMPLES_POST_TRIGGER < ADC_BUFFER_LEN){

		memcpy(&data_buffer[0],(uint8_t *)myPointerToDMA+timestamp_trigger*2-SAMPLES_PRE_TRIGGER*2,180*5);
		/*

		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2-SAMPLES_PRE_TRIGGER*2);//send pre trigger 180bytes
		HAL_Delay(10);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2);
		HAL_Delay(10);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2+180);
		HAL_Delay(10);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2+180*2);
		//HAL_Delay(10);
		//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA+timestamp_trigger*2+180*3);
		*/

		for(int i=0;i<90*5;i++){
			data_buffer[i]=transfer_to_e_calibration(data_buffer[i],0);
		}
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*2);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*3);


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

		for(int i=0;i<90*5;i++){
			data_buffer[i]=transfer_to_e_calibration(data_buffer[i],0);
		}
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
		for(int i=0;i<90*5;i++){
			data_buffer[i]=transfer_to_e_calibration(data_buffer[i],0);
		}
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*2);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*3);
		//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)data_buffer_p+180*4);
	}

}
void live_first_half(void){
	//HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
	//printf("half filled cb \r\n");
	if(myPointerToDMA!=NULL){
		uint16_t data_buffer[90];
		memcpy(&data_buffer[0],(uint8_t *)myPointerToDMA,180);
		printf("data_buffer[0]: %i\r\n",data_buffer[0]);
		if(CALIBRATED){


			for(int i=0;i<90;i++){
				data_buffer[i]=transfer_to_e_calibration(data_buffer[i],0);
			}
			//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA);
		}
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, &data_buffer[0]);
	}
}
void live_second_half(void){

	if(myPointerToDMA!=NULL){
		uint16_t data_buffer[90];
		memcpy(&data_buffer[0],(uint8_t *)myPointerToDMA+180,180);
		if(CALIBRATED){
			for(int i=0;i<90;i++){
				data_buffer[i]=transfer_to_e_calibration(data_buffer[i],0);
			}
			//Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA)+180;
		}
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, &data_buffer[0]);
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

    case CUSTOM_STM_CALIBRATION_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CALIBRATION_READ_EVT */

      /* USER CODE END CUSTOM_STM_CALIBRATION_READ_EVT */
      break;

    case CUSTOM_STM_CALIBRATION_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CALIBRATION_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_CALIBRATION_WRITE_EVT */
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
