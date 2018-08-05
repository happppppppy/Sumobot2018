
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "vl53l1_api.h"
#include "string.h"
#include "sumobot_defines.h"
#include "sumobot_functions.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId tskLASFRONTLEFTHandle;
osThreadId tskLASREARLEFTHandle;
osThreadId tskLASFRONTRIGHHandle;
osThreadId tskEDGLFTFRONTHandle;
osThreadId tskEDGLFTREARHandle;
osThreadId tskEDGRGTFRONTHandle;
osThreadId tskEDGRGTREARHandle;
osThreadId rightPWMHandle;
osThreadId leftPWMHandle;
osMutexId myMutex01Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;

VL53L1_Dev_t LASER_FRONT_LEFT_dev_t;
VL53L1_DEV LASER_FRONT_LEFT_DEV = &LASER_FRONT_LEFT_dev_t;
VL53L1_RangingMeasurementData_t RangingDataFrontLeft;

VL53L1_Dev_t LASER_FRONT_RIGHT_dev_t;
VL53L1_DEV LASER_FRONT_RIGHT_DEV = &LASER_FRONT_RIGHT_dev_t;
VL53L1_RangingMeasurementData_t RangingDataFrontRight;

VL53L1_Dev_t LASER_REAR_LEFT_dev_t;
VL53L1_DEV LASER_REAR_LEFT_DEV = &LASER_REAR_LEFT_dev_t;
VL53L1_RangingMeasurementData_t RangingDataRearLeft;

VL53L1_Dev_t LASER_REAR_RIGHT_dev_t;
VL53L1_DEV LASER_REAR_RIGHT_DEV = &LASER_REAR_RIGHT_dev_t;
VL53L1_RangingMeasurementData_t RangingDataRearRight;

VL53L1_Dev_t LASER_LEFT_FRONT_dev_t;
VL53L1_DEV LASER_LEFT_FRONT_DEV = &LASER_LEFT_FRONT_dev_t;
VL53L1_RangingMeasurementData_t RangingDataLeftFront;

VL53L1_Dev_t LASER_LEFT_REAR_dev_t;
VL53L1_DEV LASER_LEFT_REAR_DEV = &LASER_LEFT_REAR_dev_t;
VL53L1_RangingMeasurementData_t RangingDataLeftRear;

VL53L1_Dev_t LASER_RIGHT_FRONT_dev_t;
VL53L1_DEV LASER_RIGHT_FRONT_DEV = &LASER_RIGHT_FRONT_dev_t;
VL53L1_RangingMeasurementData_t RangingDataRightFront;

VL53L1_Dev_t LASER_RIGHT_REAR_dev_t;
VL53L1_DEV LASER_RIGHT_REAR_DEV = &LASER_RIGHT_REAR_dev_t;
VL53L1_RangingMeasurementData_t RangingDataRightRear;


VL53L1_DetectionConfig_t Detectionconfig;



static int status;

//Create the motors and their PID controllers
sPIDController LeftMotorPID;
sPIDController_h LeftMotorPIDHandle = &LeftMotorPID;

sPIDController RightMotorPID;
sPIDController_h RightMotorPIDHandle = &RightMotorPID;

TIM_OC_InitTypeDef LeftMotorPWMConfig;
TIM_OC_InitTypeDef RightMotorPWMConfig;

//State machine states
static char state_initialised=0;
static char state_at_centre=0;
static char state_edge_crossed=0;
static char state_opponent_lost=0;
//static char state_tracking_opponent=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void tskLASFRONTLEFT_fnc(void const * argument);
void tskLASREARLEFT_fnc(void const * argument);
void tskLASFRONTRIGHT_fnc(void const * argument);
void tskEDGLFTFRNT(void const * argument);
void tskEDGLFTRER(void const * argument);
void tskEDGRGTFRNT(void const * argument);
void tskEDGRGTRER(void const * argument);
void rightPWMFunc(void const * argument);
void leftPWMFunc(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static int SetupVL53L1XDevices(VL53L1_DEV Device_DEV, int Device_ADDRESS,
		GPIO_TypeDef* Device_SHDNPORT, uint16_t Device_SHDNPIN);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
LeftMotorPWMConfig.OCMode = LEFT_MOTOR_OCMODE;
LeftMotorPWMConfig.OCFastMode = LEFT_MOTOR_OCFAST;
LeftMotorPWMConfig.OCPolarity = LEFT_MOTOR_OCPOLARITY;
LeftMotorPWMConfig.Pulse = LEFT_MOTOR_PULSE_START;

LeftMotorPIDHandle->Kd = 0;
LeftMotorPIDHandle->Ki = 0;
LeftMotorPIDHandle->Kp = 0;

RightMotorPWMConfig.OCMode = RIGHT_MOTOR_OCMODE;
RightMotorPWMConfig.OCFastMode = RIGHT_MOTOR_OCFAST;
RightMotorPWMConfig.OCPolarity = RIGHT_MOTOR_OCPOLARITY;
RightMotorPWMConfig.Pulse = RIGHT_MOTOR_PULSE_START;

Detectionconfig.DetectionMode = VL53L1_DETECTION_DISTANCE_ONLY;
Detectionconfig.Distance.CrossMode = VL53L1_THRESHOLD_CROSSED_LOW;
Detectionconfig.IntrNoTarget = 0;
Detectionconfig.Distance.High = 1000;
Detectionconfig.Distance.Low = 500;



  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);

	//Begin setting up the distance sensors
	SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS,LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_FRONT_RIGHT_DEV,LASFRONTRIGHT_I2C_ADDRESS, LASER_FRONT_RIGHT_SHDN_GPIO_Port,LASER_FRONT_RIGHT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_REAR_LEFT_DEV, LASREARLEFT_I2C_ADDRESS,LASER_REAR_LEFT_SHDN_GPIO_Port, LASER_REAR_LEFT_SHDN_Pin);
	//	status = SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS, LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	//	status = SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS, LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	//	status = SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS, LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	//	status = SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS, LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	//	status = SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS, LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);




  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of tskLASFRONTLEFT */
  osThreadDef(tskLASFRONTLEFT, tskLASFRONTLEFT_fnc, osPriorityNormal, 0, 256);
  tskLASFRONTLEFTHandle = osThreadCreate(osThread(tskLASFRONTLEFT), NULL);

  /* definition and creation of tskLASREARLEFT */
  osThreadDef(tskLASREARLEFT, tskLASREARLEFT_fnc, osPriorityNormal, 0, 256);
  tskLASREARLEFTHandle = osThreadCreate(osThread(tskLASREARLEFT), NULL);

  /* definition and creation of tskLASFRONTRIGH */
  osThreadDef(tskLASFRONTRIGH, tskLASFRONTRIGHT_fnc, osPriorityNormal, 0, 256);
  tskLASFRONTRIGHHandle = osThreadCreate(osThread(tskLASFRONTRIGH), NULL);

  /* definition and creation of tskEDGLFTFRONT */
  osThreadDef(tskEDGLFTFRONT, tskEDGLFTFRNT, osPriorityAboveNormal, 0, 128);
  tskEDGLFTFRONTHandle = osThreadCreate(osThread(tskEDGLFTFRONT), NULL);

  /* definition and creation of tskEDGLFTREAR */
  osThreadDef(tskEDGLFTREAR, tskEDGLFTRER, osPriorityAboveNormal, 0, 128);
  tskEDGLFTREARHandle = osThreadCreate(osThread(tskEDGLFTREAR), NULL);

  /* definition and creation of tskEDGRGTFRONT */
  osThreadDef(tskEDGRGTFRONT, tskEDGRGTFRNT, osPriorityAboveNormal, 0, 128);
  tskEDGRGTFRONTHandle = osThreadCreate(osThread(tskEDGRGTFRONT), NULL);

  /* definition and creation of tskEDGRGTREAR */
  osThreadDef(tskEDGRGTREAR, tskEDGRGTRER, osPriorityAboveNormal, 0, 128);
  tskEDGRGTREARHandle = osThreadCreate(osThread(tskEDGRGTREAR), NULL);

  /* definition and creation of rightPWM */
  osThreadDef(rightPWM, rightPWMFunc, osPriorityLow, 0, 128);
  rightPWMHandle = osThreadCreate(osThread(rightPWM), NULL);

  /* definition and creation of leftPWM */
  osThreadDef(leftPWM, leftPWMFunc, osPriorityLow, 0, 128);
  leftPWMHandle = osThreadCreate(osThread(leftPWM), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while (1) {




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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967294;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LASER_RIGHT_REAR_SHDN_Pin|LASER_RIGHT_FRONT_SHDN_Pin|LASER_LEFT_REAR_SHDN_Pin|LASER_LEFT_FRONT_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LASER_REAR_RIGHT_SHDN_Pin|LASER_REAR_LEFT_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LASER_FRONT_RIGHT_SHDN_Pin|LASER_FRONT_LEFT_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EDGE_RIGHT_REAR_INTERRUPT_Pin EDGE_RIGHT_FRONT_INTERRUPT_Pin EDGE_LEFT_REAR_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = EDGE_RIGHT_REAR_INTERRUPT_Pin|EDGE_RIGHT_FRONT_INTERRUPT_Pin|EDGE_LEFT_REAR_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EDGE_LEFT_FRONT_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = EDGE_LEFT_FRONT_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EDGE_LEFT_FRONT_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_RIGHT_REAR_INTERRUPT_Pin LASER_REAR_LEFT_INTERRUPT_Pin LASER_FRONT_RIGHT_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = LASER_RIGHT_REAR_INTERRUPT_Pin|LASER_REAR_LEFT_INTERRUPT_Pin|LASER_FRONT_RIGHT_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_RIGHT_REAR_SHDN_Pin LASER_RIGHT_FRONT_SHDN_Pin LASER_LEFT_REAR_SHDN_Pin LASER_LEFT_FRONT_SHDN_Pin */
  GPIO_InitStruct.Pin = LASER_RIGHT_REAR_SHDN_Pin|LASER_RIGHT_FRONT_SHDN_Pin|LASER_LEFT_REAR_SHDN_Pin|LASER_LEFT_FRONT_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_RIGHT_FRONT_INTERRUPT_Pin LASER_LEFT_REAR_INTERRUPT_Pin LASER_LEFT_FRONT_INTERRUPT_Pin LASER_REAR_RIGHT_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = LASER_RIGHT_FRONT_INTERRUPT_Pin|LASER_LEFT_REAR_INTERRUPT_Pin|LASER_LEFT_FRONT_INTERRUPT_Pin|LASER_REAR_RIGHT_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_REAR_RIGHT_SHDN_Pin LASER_REAR_LEFT_SHDN_Pin */
  GPIO_InitStruct.Pin = LASER_REAR_RIGHT_SHDN_Pin|LASER_REAR_LEFT_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_FRONT_RIGHT_SHDN_Pin LASER_FRONT_LEFT_SHDN_Pin */
  GPIO_InitStruct.Pin = LASER_FRONT_RIGHT_SHDN_Pin|LASER_FRONT_LEFT_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_FRONT_LEFT_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = LASER_FRONT_LEFT_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LASER_FRONT_LEFT_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */


static int SetupVL53L1XDevices(VL53L1_DEV Device_DEV, int Device_ADDRESS,
		GPIO_TypeDef* Device_SHDNPORT, uint16_t Device_SHDNPIN) {

	HAL_GPIO_WritePin(Device_SHDNPORT, Device_SHDNPIN, GPIO_PIN_SET);

	Device_DEV->I2cHandle = &hi2c1;
	Device_DEV->I2cDevAddr = 0x52;
	Device_DEV->comms_speed_khz = 100;
	Device_DEV->comms_type = 1;

	status = VL53L1_WaitDeviceBooted(Device_DEV);
	status = VL53L1_SetDeviceAddress(Device_DEV, Device_ADDRESS);
	Device_DEV->I2cDevAddr = Device_ADDRESS;
	status = VL53L1_DataInit(Device_DEV);
	status = VL53L1_StaticInit(Device_DEV);
	status = VL53L1_SetDistanceMode(Device_DEV, VL53L1_DISTANCEMODE_SHORT);

	status = VL53L1_SetThresholdConfig(Device_DEV, &Detectionconfig );

	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Device_DEV, LASER_SENSOR_TIMING_BUDGET_US);
	status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Device_DEV, LASER_SENSOR_MEASUREMENT_PERIOD_MS);
	status = VL53L1_StartMeasurement(Device_DEV);

	return status;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

	  static char buffer[256];
	/* Infinite loop */
	for(;;)
	{

//		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);
		//		//	  This code changes the PWM value which sets the analogue voltage in one of the motor controllers.
		//		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
		//		HAL_TIM_PWM_ConfigChannel(&htim3, &UserPWMConfigChannel1, TIM_CHANNEL_1);
		//		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

	}
  /* USER CODE END 5 */ 
}

/* tskLASFRONTLEFT_fnc function */
void tskLASFRONTLEFT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASFRONTLEFT_fnc */

	static char buffer[256];

	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(myMutex01Handle,100);
		status = VL53L1_GetRangingMeasurementData(LASER_FRONT_LEFT_DEV,	&RangingDataFrontLeft);
		status = VL53L1_ClearInterruptAndStartMeasurement(LASER_FRONT_LEFT_DEV);

		sprintf(buffer, "{LASSenseFrontLeft:%d}\r\n",RangingDataFrontLeft.RangeMilliMeter);

		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);
		xSemaphoreGive(myMutex01Handle);
		osDelay(100);
	}
	/* USER CODE END tskLASFRONTLEFT_fnc */
}

/* tskLASREARLEFT_fnc function */
void tskLASREARLEFT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASREARLEFT_fnc */

	static char buffer[256];

	/* Infinite loop */
	for(;;)
	{

		xSemaphoreTake(myMutex01Handle,100);
		status = VL53L1_GetRangingMeasurementData(LASER_REAR_LEFT_DEV,	&RangingDataRearLeft);
		status = VL53L1_ClearInterruptAndStartMeasurement(LASER_REAR_LEFT_DEV);

		sprintf(buffer, "{LASSenseRearLeft:%d}\r\n",RangingDataRearLeft.RangeMilliMeter);

		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);
		xSemaphoreGive(myMutex01Handle);
		osDelay(100);

	}
	/* USER CODE END tskLASREARLEFT_fnc */
}

/* tskLASFRONTRIGHT_fnc function */
void tskLASFRONTRIGHT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASFRONTRIGHT_fnc */

	static char buffer[256];

	/* Infinite loop */
	for(;;)
	{

		xSemaphoreTake(myMutex01Handle,100);
		status = VL53L1_GetRangingMeasurementData(LASER_FRONT_RIGHT_DEV,	&RangingDataFrontRight);
		status = VL53L1_ClearInterruptAndStartMeasurement(LASER_FRONT_RIGHT_DEV);

		sprintf(buffer, "{LASSenseFrontRight:%d}\r\n",RangingDataFrontRight.RangeMilliMeter);

		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);
		xSemaphoreGive(myMutex01Handle);
		osDelay(100);
	}
	/* USER CODE END tskLASFRONTRIGHT_fnc */
}

/* tskEDGLFTFRNT function */
void tskEDGLFTFRNT(void const * argument)
{
  /* USER CODE BEGIN tskEDGLFTFRNT */

	  static char buffer[256];

  /* Infinite loop */
  for(;;)
  {
	  vTaskSuspend(NULL);
		sprintf(buffer, "{EDGLFTFRNT:1}\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);


  }
  /* USER CODE END tskEDGLFTFRNT */
}

/* tskEDGLFTRER function */
void tskEDGLFTRER(void const * argument)
{
  /* USER CODE BEGIN tskEDGLFTRER */

	  static char buffer[256];

  /* Infinite loop */
  for(;;)
  {
	  vTaskSuspend(NULL);
		sprintf(buffer, "{EDGLFTRER:1}\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);

  }
  /* USER CODE END tskEDGLFTRER */
}

/* tskEDGRGTFRNT function */
void tskEDGRGTFRNT(void const * argument)
{
  /* USER CODE BEGIN tskEDGRGTFRNT */

  static char buffer[256];

  /* Infinite loop */
  for(;;)
  {
	  vTaskSuspend(NULL);
		sprintf(buffer, "{EDGRGTFRNT:1}\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);

  }
  /* USER CODE END tskEDGRGTFRNT */
}

/* tskEDGRGTRER function */
void tskEDGRGTRER(void const * argument)
{
  /* USER CODE BEGIN tskEDGRGTRER */

  static char buffer[256];

  /* Infinite loop */
  for(;;)
  {
	  vTaskSuspend(NULL);
		sprintf(buffer, "{EDGRGTRER:1}\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);

  }
  /* USER CODE END tskEDGRGTRER */
}

/* rightPWMFunc function */
void rightPWMFunc(void const * argument)
{
  /* USER CODE BEGIN rightPWMFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rightPWMFunc */
}

/* leftPWMFunc function */
void leftPWMFunc(void const * argument)
{
  /* USER CODE BEGIN leftPWMFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END leftPWMFunc */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
