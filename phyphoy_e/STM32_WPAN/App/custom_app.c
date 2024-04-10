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
	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	if(*myPointerToConfigArray ==0x01){
		startADC();
	}else{
		//TODO
	}
}

void myTask(void){
	//UpdateCharData[0]+=1;
	//memcpy(&UpdateCharData[0],adc_buffer_p,20);
	if(myPointerToDMA!=NULL){
		//memcpy(&UpdateCharData[0],myPointerToDMA,20);
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)myPointerToDMA);
	}else{
		Custom_STM_App_Update_Char(CUSTOM_STM_CHANNELONE, (uint8_t *)UpdateCharData);
	}
	transferring = 0;
	//HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

	/*
	UpdateCharData[0] ^= 0x1;
	Custom_Datac_Update_Char();
	Custom_Datac_Send_Notification();
	 */

	/*
	if(!HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin)){
		HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
		UpdateCharData[0] ^= 0x1;
		Custom_Mycharnotify_Update_Char();
		Custom_Mycharnotify_Send_Notification();
	}
	*/

	//memcpy(&UpdateCharData[0],dma_adc,20);
	//Custom_STM_App_Update_Char(CUSTOM_STM_DATAC, (uint8_t *)UpdateCharData);

	//UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);


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
