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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "string.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

/* Definitions for USB_Listen */
osThreadId_t USB_ListenHandle;
const osThreadAttr_t USB_Listen_attributes = {
  .name = "USB_Listen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CheckZVS */
osThreadId_t CheckZVSHandle;
const osThreadAttr_t CheckZVS_attributes = {
  .name = "CheckZVS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDStandby */
osThreadId_t LEDStandbyHandle;
const osThreadAttr_t LEDStandby_attributes = {
  .name = "LEDStandby",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SelfTest */
osThreadId_t SelfTestHandle;
const osThreadAttr_t SelfTest_attributes = {
  .name = "SelfTest",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ProcessADC */
osThreadId_t ProcessADCHandle;
const osThreadAttr_t ProcessADC_attributes = {
  .name = "ProcessADC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LED_Control */
osThreadId_t LED_ControlHandle;
const osThreadAttr_t LED_Control_attributes = {
  .name = "LED_Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for FiringSeq */
osThreadId_t FiringSeqHandle;
const osThreadAttr_t FiringSeq_attributes = {
  .name = "FiringSeq",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ZVStoVcap */
osMessageQueueId_t ZVStoVcapHandle;
const osMessageQueueAttr_t ZVStoVcap_attributes = {
  .name = "ZVStoVcap"
};
/* Definitions for ISRtoProcessADC */
osMessageQueueId_t ISRtoProcessADCHandle;
const osMessageQueueAttr_t ISRtoProcessADC_attributes = {
  .name = "ISRtoProcessADC"
};
/* Definitions for ProcessADCtoLED */
osMessageQueueId_t ProcessADCtoLEDHandle;
const osMessageQueueAttr_t ProcessADCtoLED_attributes = {
  .name = "ProcessADCtoLED"
};
/* Definitions for FiringData */
osMessageQueueId_t FiringDataHandle;
const osMessageQueueAttr_t FiringData_attributes = {
  .name = "FiringData"
};
/* Definitions for LEDs */
osMutexId_t LEDsHandle;
const osMutexAttr_t LEDs_attributes = {
  .name = "LEDs"
};
/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
void USB_Listen_Start(void *argument);
void StartCheckZVS(void *argument);
void StartLEDStandby(void *argument);
void StartSelfTest(void *argument);
void StartProcessADC(void *argument);
void StartLED_Control(void *argument);
void StartFiringSeq(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buffer[64];
uint8_t rec_cplt;
uint8_t trig_count=0;
uint32_t ADC_raw[2];
uint16_t c_delay=0;
uint32_t Vcap_prev=0;

#define ERR_OVER_CHRG 1;

void delay(uint16_t dely){

	__HAL_TIM_SET_COUNTER(&htim3,0);
	while(__HAL_TIM_GET_COUNTER(&htim3)<dely);

}

void LED_Reset(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
}

void LED_Follow(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	osDelay(100);
}

void LED_Flash(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	osDelay(1000);

}

void chrg_led_reset(void){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
}

void overcharge(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	osDelay(100);

}

void led_quick_flash(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	osDelay(100);
}

int Charge_val_round(uint32_t ADC_val){
	if(ADC_val<100){
		ADC_val=100;
	}

	if(ADC_val>100 && ADC_val<250){
		ADC_val=250;
	}

	if(ADC_val>250 && ADC_val<385){
		ADC_val=385;
	}

	if(ADC_val>385){
		ADC_val=400;
	}

	return(ADC_val);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(trig_count==1){
		return;
	}
	if(GPIO_Pin==GPIO_PIN_10){
		trig_count=1;


	}

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_ADC_Start_DMA(&hadc1, ADC_raw, 2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(ADC_raw[0]>Vcap_prev){
		Vcap_prev=ADC_raw[0];
	}
	BaseType_t xYieldRequired;
	xQueueSendFromISR(ISRtoProcessADCHandle,&Vcap_prev,NULL);
	xQueueSendFromISR(ISRtoProcessADCHandle,&ADC_raw[1],NULL);
	xYieldRequired = xTaskResumeFromISR(ProcessADCHandle);
	portYIELD_FROM_ISR( xYieldRequired );
}

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
  TIM4->CCR1=2000-1;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of LEDs */
  LEDsHandle = osMutexNew(&LEDs_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  xSemaphoreGive(LEDsHandle);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ZVStoVcap */
  ZVStoVcapHandle = osMessageQueueNew (1, sizeof(uint32_t), &ZVStoVcap_attributes);

  /* creation of ISRtoProcessADC */
  ISRtoProcessADCHandle = osMessageQueueNew (2, sizeof(uint32_t), &ISRtoProcessADC_attributes);

  /* creation of ProcessADCtoLED */
  ProcessADCtoLEDHandle = osMessageQueueNew (1, sizeof(uint32_t), &ProcessADCtoLED_attributes);

  /* creation of FiringData */
  FiringDataHandle = osMessageQueueNew (2, sizeof(uint32_t), &FiringData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of USB_Listen */
  USB_ListenHandle = osThreadNew(USB_Listen_Start, NULL, &USB_Listen_attributes);

  /* creation of CheckZVS */
  CheckZVSHandle = osThreadNew(StartCheckZVS, NULL, &CheckZVS_attributes);

  /* creation of LEDStandby */
  LEDStandbyHandle = osThreadNew(StartLEDStandby, NULL, &LEDStandby_attributes);

  /* creation of SelfTest */
  SelfTestHandle = osThreadNew(StartSelfTest, NULL, &SelfTest_attributes);

  /* creation of ProcessADC */
  ProcessADCHandle = osThreadNew(StartProcessADC, NULL, &ProcessADC_attributes);

  /* creation of LED_Control */
  LED_ControlHandle = osThreadNew(StartLED_Control, NULL, &LED_Control_attributes);

  /* creation of FiringSeq */
  FiringSeqHandle = osThreadNew(StartFiringSeq, NULL, &FiringSeq_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 42000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 200-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ZVS_ON_GPIO_Port, ZVS_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14
                           PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ZVS_Pin Safety_Pin */
  GPIO_InitStruct.Pin = ZVS_Pin|Safety_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Interupt_Pin */
  GPIO_InitStruct.Pin = Trigger_Interupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Trigger_Interupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ZVS_ON_Pin */
  GPIO_InitStruct.Pin = ZVS_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ZVS_ON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_USB_Listen_Start */
/**
  * @brief  Function implementing the USB_Listen thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_USB_Listen_Start */
void USB_Listen_Start(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  HAL_TIM_Base_Start(&htim3);
  /* Infinite loop */
  for(;;)
  {
	  char mess[64];

	  if(rec_cplt==1){
		  if(strcmp(buffer,"C1")==0){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
			  HAL_Delay(1000);
			  rec_cplt=0;
		  }

		  else{
			  c_delay=atoi(buffer);
			  sprintf(mess,"Delay set to %i us",c_delay);
			  CDC_Transmit_FS(mess, strlen(mess));
		  }





	  }
	  vTaskSuspend(NULL);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCheckZVS */
/**
* @brief Function implementing the CheckZVS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCheckZVS */
void StartCheckZVS(void *argument)
{
  /* USER CODE BEGIN StartCheckZVS */
  /* Infinite loop */
  for(;;)
  {

	 if(trig_count==0){
		 vTaskSuspend(NULL);
	 }
	 if(trig_count==1){

		 if(ulTaskNotifyTake(pdTRUE, 0)==1){

		 }

		 else{
			 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==0){

			 					 osDelay(2000);
			 					 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==0){
			 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			 						osDelay(c_delay);
			 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
			 						osDelay(100);
			 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			 						osDelay(10);
			 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
			 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
			 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);
			 					    vTaskSuspend(NULL); //Resumed in Vcap
			 					 	}



			 			 	}
		 }
		 }

	 }


  /* USER CODE END StartCheckZVS */
}

/* USER CODE BEGIN Header_StartLEDStandby */
/**
* @brief Function implementing the LEDStandby thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDStandby */
void StartLEDStandby(void *argument)
{
  /* USER CODE BEGIN StartLEDStandby */
  /* Infinite loop */
  for(;;)
  {

		 LED_Flash();

  }
  /* USER CODE END StartLEDStandby */
}

/* USER CODE BEGIN Header_StartSelfTest */
/**
* @brief Function implementing the SelfTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSelfTest */
void StartSelfTest(void *argument)
{
  /* USER CODE BEGIN StartSelfTest */
  /* Infinite loop */
  for(;;)
  {

    if(trig_count==1){
    	vTaskSuspend(LEDStandbyHandle);
    	xSemaphoreTake(LEDsHandle,portMAX_DELAY);
    	LED_Reset();
    	LED_Follow();
    	LED_Follow();
    	LED_Follow();
    	xSemaphoreGive(LEDsHandle);
    	vTaskResume(LED_ControlHandle);
    	vTaskResume(CheckZVSHandle);
    	vTaskResume(FiringSeqHandle);
    	vTaskSuspend(NULL);
    }
    osDelay(1);
  }
  /* USER CODE END StartSelfTest */
}

/* USER CODE BEGIN Header_StartProcessADC */
/**
* @brief Function implementing the ProcessADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessADC */
void StartProcessADC(void *argument)
{
  /* USER CODE BEGIN StartProcessADC */
  /* Infinite loop */
  for(;;)
  {

	if(trig_count==1){
		vTaskResume(LED_ControlHandle);
	}
	uint32_t ADC_vals[2];
	int Vcap;
	int Charge_val;
    xQueueReceive(ISRtoProcessADCHandle, &ADC_vals[0], 0); //Receive Capacitor voltage
    xQueueReceive(ISRtoProcessADCHandle, &ADC_vals[1], 0); //Receive Capacitor voltage
    Vcap=((float)ADC_vals[0]/4095)*729.3;
    Charge_val=((float)ADC_vals[1]/4095)*400*(-1)+400;
    Charge_val=Charge_val_round(Charge_val);

    if(Vcap==Charge_val || Vcap>Charge_val){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
    	vTaskResume(FiringSeqHandle);

    }

    CDC_Transmit_FS(&Vcap, 2);
    osDelay(100);
    xQueueSend(ProcessADCtoLEDHandle,&Charge_val,0);
    vTaskSuspend(NULL);


  }
  /* USER CODE END StartProcessADC */
}

/* USER CODE BEGIN Header_StartLED_Control */
/**
* @brief Function implementing the LED_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED_Control */
void StartLED_Control(void *argument)
{
  /* USER CODE BEGIN StartLED_Control */
  /* Infinite loop */
  for(;;)
  {
	int Charge_val=0;
	xQueueReceive(ProcessADCtoLEDHandle, &Charge_val, 0);
    if(trig_count==0){
    	vTaskSuspend(NULL);
    }
    xSemaphoreTake(LEDsHandle,portMAX_DELAY); //Take control of LED GPIOs

       	if(Charge_val==100){
       		    HAL_TIM_Base_Stop_IT(&htim10);
       	    	chrg_led_reset();
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
       	    	vTaskResume(CheckZVSHandle);
       	    }

       	if(Charge_val==250){
       		    HAL_TIM_Base_Stop_IT(&htim10);
       		    chrg_led_reset();
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
       	    	vTaskResume(CheckZVSHandle);
       	    }

       	if(Charge_val==385){
       		    HAL_TIM_Base_Stop_IT(&htim10);
       		    chrg_led_reset();
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
       	    	vTaskResume(CheckZVSHandle);
       	    }

       if(Charge_val>385){
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
       	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
       	    	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==0){
       	    		HAL_TIM_Base_Stop_IT(&htim10);
       	    		LED_Reset();
       	    		led_quick_flash();
       	    		led_quick_flash();
       	    		led_quick_flash();
       	    	}
       	    	else{
       	    		HAL_TIM_Base_Start_IT(&htim10);
       	    	}

       	    }
       xSemaphoreGive(LEDsHandle); //Release control of LED GPIOs
       vTaskSuspend(NULL);



  }
  /* USER CODE END StartLED_Control */
}

/* USER CODE BEGIN Header_StartFiringSeq */
/**
* @brief Function implementing the FiringSeq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFiringSeq */
void StartFiringSeq(void *argument)
{
  /* USER CODE BEGIN StartFiringSeq */
  /* Infinite loop */
  for(;;)
  {
    if(trig_count==0){
    	vTaskSuspend(NULL);
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0); //Turn off ZVS





    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

    	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==0){ //Check Trigger
    	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
    	    	delay(c_delay);
    	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
    	    	osDelay(100);
    	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
    	    	osDelay(10);
    	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
    	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
    	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);
    	    	Vcap_prev=0;
    	    	osDelay(100);
    	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
    	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
    	}





    vTaskResume(CheckZVSHandle); //Resume checking of ZVS
    vTaskSuspend(NULL);
  }
  /* USER CODE END StartFiringSeq */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if(htim->Instance== TIM10){
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

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
