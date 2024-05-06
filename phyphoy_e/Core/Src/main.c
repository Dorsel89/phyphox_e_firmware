/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "dac.h"
#include "shared.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t flash_address = 0x08080000;

uint8_t SAMPLETIME[8]={ADC_SAMPLETIME_2CYCLES_5,	// 0
		  ADC_SAMPLETIME_6CYCLES_5,					// 1
		  ADC_SAMPLETIME_12CYCLES_5,				// 2
		  ADC_SAMPLETIME_24CYCLES_5,				// 3
		  ADC_SAMPLETIME_47CYCLES_5,				// 4
		  ADC_SAMPLETIME_92CYCLES_5,				// 5
		  ADC_SAMPLETIME_247CYCLES_5,				// 6g
		  ADC_SAMPLETIME_640CYCLES_5};
float SAMPLETIME_CYCLES[8]={2.5,	// 0
		  6.5,					// 1
		  12.5,				// 2
		  24.5,				// 3
		  47.5,				// 4
		  92.5,				// 5
		  247.5,				// 6g
		  640.5};

uint8_t PRESCALER[]={ADC_CLOCK_ASYNC_DIV1,
					ADC_CLOCK_ASYNC_DIV2,
					ADC_CLOCK_ASYNC_DIV4,
					ADC_CLOCK_ASYNC_DIV8,
					ADC_CLOCK_ASYNC_DIV16,
					ADC_CLOCK_ASYNC_DIV32
};
uint8_t PRESCALER_DIVIDER[]={1,
					2,
					4,
					8,
					16,
					32
};
uint8_t OVERSAMPLING[]={ADC_OVERSAMPLING_RATIO_2,
						ADC_OVERSAMPLING_RATIO_2,
						ADC_OVERSAMPLING_RATIO_4,
						ADC_OVERSAMPLING_RATIO_8,
						ADC_OVERSAMPLING_RATIO_16,
						ADC_OVERSAMPLING_RATIO_32,
						ADC_OVERSAMPLING_RATIO_64,
						ADC_OVERSAMPLING_RATIO_128,
						ADC_OVERSAMPLING_RATIO_256
};
uint16_t OVERSAMPLING_DIVIDER[9]={1,
						2,
						4,
						8,
						16,
						32,
						64,
						128,
						256
};
uint8_t BITSHIFT[]={ADC_RIGHTBITSHIFT_NONE,
						ADC_RIGHTBITSHIFT_1,
						ADC_RIGHTBITSHIFT_2,
						ADC_RIGHTBITSHIFT_3,
						ADC_RIGHTBITSHIFT_4,
						ADC_RIGHTBITSHIFT_5,
						ADC_RIGHTBITSHIFT_6,
						ADC_RIGHTBITSHIFT_7,
						ADC_RIGHTBITSHIFT_8
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp1;

I2C_HandleTypeDef hi2c1;

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUFFER_LEN];

float dac_voltage[2]={0};

uint8_t* adc_mode = &adc_config[0];
uint8_t* adc_routing = &adc_config[1];
uint8_t* adc_sampletime = &adc_config[2];
uint8_t* adc_clock_prescaler = &adc_config[3];
uint8_t* adc_oversampling = &adc_config[4];
uint8_t* adc_edge = &adc_config[5];

extern uint8_t ble_flag;
extern uint8_t ble_buffer[1];
extern volatile uint8_t transferring = 0;
extern uint8_t adc_char_length = 180;
extern volatile uint8_t activate_trigger = 0;

volatile int Is_First_Captured = 0;
volatile int first_adc_loop_finished = 0;

volatile uint32_t IC_Val1 = 0;
volatile uint16_t timer_val;

volatile uint32_t reference_counts = 0;

extern volatile uint16_t timestamp_trigger = 0;
extern volatile uint16_t timestamp_adc_stop = 0;

extern volatile uint16_t SAMPLES_PRE_TRIGGER = 90;
extern volatile uint16_t SAMPLES_POST_TRIGGER = 270;
extern volatile uint16_t my_prescaler = 30;//16417;


extern float CALI_DAC_INT[2] = {0,0};

extern float CALI_LOW_FLOAT[2] = {0.0,0.0};
extern int CALI_LOW_INT[2] = {2029,2029};
extern float CALI_HIGH_FLOAT[2] = {10.02,10.02};
extern int CALI_HIGH_INT[2] = {3472,3472};
extern uint8_t CALIBRATED = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_COMP1_Init(void);
static void MX_ADC1_Init(void);
static void MX_IPCC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t write_i2c(uint8_t addr, uint16_t reg, uint8_t *data_w, uint16_t len);
static uint8_t read_i2c(uint8_t addr, uint16_t reg, uint8_t *data_r, uint16_t len);
uint16_t volatile timer_val;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len){
	int i=0;
	for(i=0; i<len; i++){
		ITM_SendChar((*ptr++));
	}
	return len;
}

extern dacx3202_t dacx3202 = {.addr = DACX3202_7B_ADDR(0x00),
							.i2c_read = read_i2c,
							.i2c_write = write_i2c,
							.vref = 3.0};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_COMP1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
  adc_buffer_p = &adc_buf[0];

//HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_1);


  //HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

  //HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);

  HAL_COMP_Start(&hcomp1);


  dacx3202_init(&dacx3202);
  dacx3202_power_up(&dacx3202, DACX3202_DAC_0);
  dacx3202_set_voltage(&dacx3202, DACX3202_DAC_0, 1.59);
  dacx3202_power_up(&dacx3202, DACX3202_DAC_1);
  //dacx3202_set_voltage(&dacx3202, DACX3202_DAC_1, 2.2);
  dacx3202_set_value(&dacx3202, DACX3202_DAC_1, 0);

  myPointerToDMA = &adc_buf[0];
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)== HAL_OK){
	  //HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  }
  //HAL_ADC_Start_IT(&hadc1);

  //start_circular_adc();


  //HAL_ADC_Start_DMA_(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  uint16_t dac_calibration_value = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  printf("dac_calibration_valuie low%i\r\n",dac_calibration_value);
  CALI_DAC_INT[0]=dac_calibration_value;

  dacx3202_set_value(&dacx3202, DACX3202_DAC_1, 1023);
  HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	dac_calibration_value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	printf("dac_calibration_valuie high %i\r\n",dac_calibration_value);
	CALI_DAC_INT[1]=dac_calibration_value;

/*
  HAL_FLASH_Unlock();
  uint64_t test =3;
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_address, test);
  HAL_FLASH_Lock();






  */
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_IO3;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 30;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000+500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim1, TIM_TIM1_TI1_COMP1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 30;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);


  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(switchRange_GPIO_Port, switchRange_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_R_Pin|LED_B_Pin|LED_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUXA1_GPIO_Port, MUXA1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUXA0_GPIO_Port, MUXA0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : in_dac_trigger_Pin */
  GPIO_InitStruct.Pin = in_dac_trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(in_dac_trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : switchRange_Pin LED_R_Pin LED_B_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = switchRange_Pin|LED_R_Pin|LED_B_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MUXA1_Pin MUXA0_Pin */
  GPIO_InitStruct.Pin = MUXA1_Pin|MUXA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GROUND_Pin */
  GPIO_InitStruct.Pin = GROUND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GROUND_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}

static uint8_t write_i2c(uint8_t addr, uint16_t reg, uint8_t *data_w, uint16_t len){
	HAL_StatusTypeDef ret;
	const int cLen = len + 1;
	uint8_t tx[cLen];
	tx[0] = reg;
	for (int i = 0; i < len; i++) {
	        tx[i + 1] = data_w[i];
	}
	ret = HAL_I2C_Master_Transmit(&hi2c1, addr, tx , cLen, HAL_MAX_DELAY);
	if(ret){
		//HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);
	}

}

static uint8_t read_i2c(uint8_t addr, uint16_t reg, uint8_t *data_r, uint16_t len){
	HAL_StatusTypeDef ret;
	uint8_t tx[1] = {reg};

	ret = HAL_I2C_Master_Transmit(&hi2c1, addr, &tx[0] , 1, HAL_MAX_DELAY);

	if(ret){
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, addr, data_r, len, HAL_MAX_DELAY);

	if(ret){
		//HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);
	}

}

extern void change_edge(uint8_t e){
	TIM_IC_InitTypeDef sConfigIC = {0};

	if(e==0){
		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	}else if(e==1){
		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	}else if(e==2){
		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	}

	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 10;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler();
	}
}
extern void set_dac(float val){
	  dacx3202_set_voltage(&dacx3202, DACX3202_DAC_1, val);
}

extern void start_circular_adc(){

	//init timer

	//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_Base_Stop(&htim2);

	/*

	htim1.Init.Prescaler = my_prescaler-1;
	htim2.Init.Prescaler = my_prescaler-1;


	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
	Error_Handler();
	}

	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE_IT(&htim2,TIM_IT_UPDATE);

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
	Error_Handler();
	}


	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);
	*/
	//__HAL_TIM_SET_PRESCALER(&htim1,my_prescaler-1);
	//__HAL_TIM_SET_PRESCALER(&htim2,my_prescaler-1);

	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
	//starting everything
	Is_First_Captured = 0;
	first_adc_loop_finished = 0;
	printf("activate trigger! \r\n");
	//HAL_TIM_Base_Start_IT(&htim2);
	//htim2.Instance->CNT = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);

}

extern void startADC(){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
	//timer_val = __HAL_TIM_GET_COUNTER(&htim17);
	//HAL_Delay(2);
	//timer_val = __HAL_TIM_GET_COUNTER(&htim17) - timer_val;
	//printf("delay: %u \r\n", timer_val);
}

extern void set_dma_circular(uint8_t b){
	if(b==1){
		hdma_adc1.Init.Mode = DMA_CIRCULAR;//DMA_NORMAL
	}else{
		hdma_adc1.Init.Mode = DMA_NORMAL;
	}
}

extern void new_adc_init(){

	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_DeInit(&hadc1);



	memcpy(&dac_voltage[1],&adc_config[6],4);
	int dac_val_nc = ((dac_voltage[1]*1443/10.02)+2029);
	printf("dac_val_nc %i  CALI_DAC_INT 0 %i  CALI_DAC_INT 1 %i\r\n",dac_val_nc,CALI_DAC_INT[0],CALI_DAC_INT[1]);
	int dac_val = CALI_DAC_INT[0]+(1023/(CALI_DAC_INT[1]-CALI_DAC_INT[0]))*dac_val_nc;
	printf("dac value: %i\r\n",dac_val);
	dacx3202_set_value(&dacx3202, DACX3202_DAC_1, dac_val);

	//dacx3202_set_voltage(&dacx3202, DACX3202_DAC_0, my_dac_val);

	//disable dac for now!
	//set_dac(my_dac_val);
	change_edge(*adc_edge);
	//if(adc_mode == 1){
	printf("adc mode: %i\r\n",*adc_mode);
	printf("routing: %i\r\n",*adc_routing);
	printf("adc_sampletime: %i\r\n",*adc_sampletime);
	printf("adc_clock_prescaler: %i\r\n",*adc_clock_prescaler);
	printf("adc_oversampling: %i\r\n",*adc_oversampling);
	printf("adc_edge: %i\r\n",*adc_edge);

	printf("0: %i ,1: %i ,2: %i ,3: %i, \r\n",adc_config[5],adc_config[6],adc_config[7],adc_config[8]);

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	//hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
	if(*adc_clock_prescaler ==0){
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	}else if(*adc_clock_prescaler ==1){
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	}else if(*adc_clock_prescaler ==2){
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
	}else if(*adc_clock_prescaler ==3){
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
	}else if(*adc_clock_prescaler ==4){
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
	}else if(*adc_clock_prescaler ==5){
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
	}
	//hadc1.Init.ClockPrescaler = PRESCALER[*adc_clock_prescaler];
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	if(*adc_mode == 1){
		//hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
		//hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
		hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	}else if(*adc_mode == 0){
		hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		//hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	}
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	//hadc1.Init.OversamplingMode = ENABLE;
	//hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
	//hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
	//hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;

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


	hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
	Error_Handler();
	}

	/** Configure Regular Channel
	*/
	if(*adc_routing == 1){
		sConfig.Channel = ADC_CHANNEL_14;
	}else if(*adc_routing == 2){
		sConfig.Channel = ADC_CHANNEL_2;
	}
	//sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	//sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SamplingTime = SAMPLETIME[*adc_sampletime];
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	if(*adc_mode == 1){
		//oscillator mode
		float prescaler_f = ((12.5 + SAMPLETIME_CYCLES[*adc_sampletime])*OVERSAMPLING_DIVIDER[*adc_oversampling])*PRESCALER_DIVIDER[*adc_clock_prescaler]/(2.0);
		my_prescaler = prescaler_f;
		printf("OVERSAMPLING_DIVIDER: %i\r\n",OVERSAMPLING_DIVIDER[*adc_oversampling]);

		printf("OVERSAMPLING_DIVIDER[*adc_oversampling] %i\r\n",OVERSAMPLING_DIVIDER[*adc_oversampling]);
		printf("SAMPLETIME_CYCLES: %i\r\n",(uint32_t)(OVERSAMPLING_DIVIDER[*adc_oversampling]*SAMPLETIME_CYCLES[*adc_sampletime]));
		printf("PRESCALER_DIVIDER: %i\r\n",PRESCALER_DIVIDER[*adc_clock_prescaler]);
		printf("my_prescaler: %i\r\n",my_prescaler);
		//__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		//__HAL_TIM_DISABLE_IT(&htim2,TIM_IT_UPDATE);
		__HAL_TIM_SET_PRESCALER(&htim1,my_prescaler);
		__HAL_TIM_SET_PRESCALER(&htim2,my_prescaler);
		//htim1.Instance->PSC = my_prescaler;
		//htim2.Instance->PSC = my_prescaler;
		htim2.Instance->CNT = 0;
		htim1.Instance->CNT = 0;

		//__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		//__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);
	}

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	if(*adc_mode == 0){
		printf("start live mode!\r\n");
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 180);
	}
}


void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){
//	printf("comp call  \r\n");
}


//Callback when half filled

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	/*
	if (htim == &htim2){
		timer_val = __HAL_TIM_GET_COUNTER(&htim1);
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_TIM_Base_Stop_IT(&htim2);
		htim2.Instance->CNT = 0;
		timestamp_adc_stop = timer_val;//ADC_BUFFER_LEN*timer_val/(reference_counts);
		//printf("trigger is no %i , ",timestamp_trigger);
		printf(" end is at %i , reference counts: %i\r\n",timestamp_adc_stop,reference_counts);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
		UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);
	}
	*/
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	//timer_val = __HAL_TIM_GET_COUNTER(&htim17);
	//UTIL_SEQ_SetTask(1 << CFG_TASK_HALF_FILLED, CFG_SCH_PRIO_0);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//	HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
	//printf("got tim evenz \r\n");

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		if (Is_First_Captured==0 && first_adc_loop_finished){
			Is_First_Captured = 1;  // set the first captured as true
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			while(htim1.Instance->CNT < SAMPLES_POST_TRIGGER + IC_Val1 + 10);
			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
			timestamp_trigger = IC_Val1*ADC_BUFFER_LEN/(reference_counts);
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
			UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);
		}
	}
}
//Callback when buffer filled

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	/*
	reference_counts = __HAL_TIM_GET_COUNTER(&htim1);
	htim1.Instance->CNT = 0;
	if(first_adc_loop_finished == 0){
		first_adc_loop_finished = 1;
	}
	*/
	if(*adc_mode == 1){
		reference_counts = __HAL_TIM_GET_COUNTER(&htim1);
		htim1.Instance->CNT = 0;
		if(first_adc_loop_finished == 0){
			first_adc_loop_finished = 1;
		}
		printf("adc callback %i \r\n",reference_counts);
	}else if(*adc_mode == 0){
		UTIL_SEQ_SetTask(1 << CFG_TASK_FILLED, CFG_SCH_PRIO_0);
	}
	//printf("buff full! reference counts: %i \r\n",reference_counts);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
