
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
#include "sumobot_functions.h"
#include "TMC5160.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId tskLASFRONTLEFTHandle;
osThreadId tskLASREARLEFTHandle;
osThreadId tskLASFRONTRIGHHandle;
osThreadId tskLASREARRIGHTHandle;
osThreadId tskLASLEFTFRONTHandle;
osThreadId tskLASLEFTREARHandle;
osThreadId tskLASRIGHTFRONHandle;
osThreadId tskLASRIGHTREARHandle;
osThreadId tskEDGEHandle;
osMutexId I2CMutexHandle;
osMutexId SPIMutexHandle;

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

//State machine states
static char state_seeking=0;
static char state_tracking=0;
static char state_start_fight=1;

static unsigned char state_edges= 0x0; //Bit 0:Edge left front Bit 1:Edge left rear Bit 2:Edge right front Bit 3: Edge right rear

//SPI bus variables
uint8_t spiTXbuffer[5], spiRXbuffer[5];
uint8_t SPISTATUS;

//Motors
sTMC5160Motor left_stepper;
sTMC5160Motor right_stepper;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void tskLASFRONTLEFT_fnc(void const * argument);
void tskLASREARLEFT_fnc(void const * argument);
void tskLASFRONTRIGHT_fnc(void const * argument);
void tskLASREARRIGHT_fnc(void const * argument);
void tskLASLEFTFRONT_fnc(void const * argument);
void tskLASLEFTREAR_fnc(void const * argument);
void tskLASRIGHTFRONT_fnc(void const * argument);
void tskLASRIGHTREAR_fnc(void const * argument);
void tskEDGE_fnc(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//All private functions in sumobot_functions.c, sumobot_functions.h
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

	Detectionconfig.DetectionMode = VL53L1_DETECTION_NORMAL_RUN;
	Detectionconfig.Distance.CrossMode = VL53L1_THRESHOLD_IN_WINDOW;
	Detectionconfig.IntrNoTarget = 0;
	Detectionconfig.Distance.High = LASER_SENSOR_HIGH_THRESHOLD;
	Detectionconfig.Distance.Low = LASER_SENSOR_LOW_THRESHOLD;


	left_stepper.acceleration_1 = LM_A1;
	left_stepper.acceleration_max = LM_AMAX;
	left_stepper.deceleration_1 = LM_D1;
	left_stepper.deceleration_max = LM_DMAX;
	left_stepper.velocity_1 = LM_V1;
	left_stepper.velocity_max = LM_VMAX;
	left_stepper.velocity_start = LM_VSTART;
	left_stepper.velocity_stop = LM_VSTOP;
	left_stepper.gpio_port = Stepper_Left_CSN_GPIO_Port;
	left_stepper.gpio_pin = Stepper_Left_CSN_Pin;
	left_stepper.direction = LM_DIRECTION;


	right_stepper.acceleration_1 = LM_A1;
	right_stepper.acceleration_max = LM_AMAX;
	right_stepper.deceleration_1 = LM_D1;
	right_stepper.deceleration_max = LM_DMAX;
	right_stepper.velocity_1 = LM_V1;
	right_stepper.velocity_max = LM_VMAX;
	right_stepper.velocity_start = LM_VSTART;
	right_stepper.velocity_stop = LM_VSTOP;
	right_stepper.gpio_port = Stepper_Right_CSN_GPIO_Port;
	right_stepper.gpio_pin = Stepper_Right_CSN_Pin;
	right_stepper.direction = RM_DIRECTION;



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
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */


	//Initialise the distance sensors
	SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS,LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_FRONT_RIGHT_DEV,LASFRONTRIGHT_I2C_ADDRESS, LASER_FRONT_RIGHT_SHDN_GPIO_Port,LASER_FRONT_RIGHT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_REAR_LEFT_DEV, LASREARLEFT_I2C_ADDRESS,LASER_REAR_LEFT_SHDN_GPIO_Port, LASER_REAR_LEFT_SHDN_Pin);
	//	SetupVL53L1XDevices(LASER_REAR_RIGHT_DEV, LASREARRIGHT_I2C_ADDRESS, LASER_REAR_RIGHT_SHDN_GPIO_Port, LASER_REAR_RIGHT_SHDN_Pin);
	//	SetupVL53L1XDevices(LASER_LEFT_FRONT_DEV, LASLEFTFRONT_I2C_ADDRESS, LASER_LEFT_FRONT_SHDN_GPIO_Port, LASER_LEFT_FRONT_SHDN_Pin);
	//	SetupVL53L1XDevices(LASER_LEFT_REAR_DEV, LASLEFTREAR_I2C_ADDRESS, LASER_LEFT_REAR_SHDN_GPIO_Port, LASER_LEFT_REAR_SHDN_Pin);
	//	SetupVL53L1XDevices(LASER_RIGHT_FRONT_DEV, LASRIGHTFRONT_I2C_ADDRESS, LASER_RIGHT_FRONT_SHDN_GPIO_Port, LASER_RIGHT_FRONT_SHDN_Pin);
	//	SetupVL53L1XDevices(LASER_RIGHT_REAR_DEV, LASRIGHTREAR_I2C_ADDRESS, LASER_RIGHT_REAR_SHDN_GPIO_Port, LASER_RIGHT_REAR_SHDN_Pin);

	//Initialise the motors
	if(LEFT_MOTOR_ENABLED){
		tmc5160_initialise_motor(left_stepper);
	}
	if(RIGHT_MOTOR_ENABLED){
		tmc5160_initialise_motor(right_stepper);
	}

	state_edges= 0x0;

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of I2CMutex */
	osMutexDef(I2CMutex);
	I2CMutexHandle = osMutexCreate(osMutex(I2CMutex));

	/* definition and creation of SPIMutex */
	osMutexDef(SPIMutex);
	SPIMutexHandle = osMutexCreate(osMutex(SPIMutex));

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

	/* definition and creation of tskLASREARRIGHT */
	osThreadDef(tskLASREARRIGHT, tskLASREARRIGHT_fnc, osPriorityNormal, 0, 256);
	tskLASREARRIGHTHandle = osThreadCreate(osThread(tskLASREARRIGHT), NULL);

	/* definition and creation of tskLASLEFTFRONT */
	osThreadDef(tskLASLEFTFRONT, tskLASLEFTFRONT_fnc, osPriorityNormal, 0, 256);
	tskLASLEFTFRONTHandle = osThreadCreate(osThread(tskLASLEFTFRONT), NULL);

	/* definition and creation of tskLASLEFTREAR */
	osThreadDef(tskLASLEFTREAR, tskLASLEFTREAR_fnc, osPriorityNormal, 0, 256);
	tskLASLEFTREARHandle = osThreadCreate(osThread(tskLASLEFTREAR), NULL);

	/* definition and creation of tskLASRIGHTFRON */
	osThreadDef(tskLASRIGHTFRON, tskLASRIGHTFRONT_fnc, osPriorityNormal, 0, 256);
	tskLASRIGHTFRONHandle = osThreadCreate(osThread(tskLASRIGHTFRON), NULL);

	/* definition and creation of tskLASRIGHTREAR */
	osThreadDef(tskLASRIGHTREAR, tskLASRIGHTREAR_fnc, osPriorityNormal, 0, 256);
	tskLASRIGHTREARHandle = osThreadCreate(osThread(tskLASRIGHTREAR), NULL);

	/* definition and creation of tskEDGE */
	osThreadDef(tskEDGE, tskEDGE_fnc, osPriorityAboveNormal, 0, 128);
	tskEDGEHandle = osThreadCreate(osThread(tskEDGE), NULL);

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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

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
	HAL_GPIO_WritePin(GPIOC, Stepper_Left_CSN_Pin|LASER_REAR_RIGHT_SHDN_Pin|LASER_REAR_LEFT_SHDN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LASER_RIGHT_REAR_SHDN_Pin|LASER_RIGHT_FRONT_SHDN_Pin|LASER_LEFT_REAR_SHDN_Pin|LASER_LEFT_FRONT_SHDN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Stepper_Right_CSN_Pin|LASER_FRONT_RIGHT_SHDN_Pin|LASER_FRONT_LEFT_SHDN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : EDGE_RIGHT_REAR_INTERRUPT_Pin EDGE_RIGHT_FRONT_INTERRUPT_Pin EDGE_LEFT_REAR_INTERRUPT_Pin */
	GPIO_InitStruct.Pin = EDGE_RIGHT_REAR_INTERRUPT_Pin|EDGE_RIGHT_FRONT_INTERRUPT_Pin|EDGE_LEFT_REAR_INTERRUPT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : Stepper_Left_CSN_Pin */
	GPIO_InitStruct.Pin = Stepper_Left_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Stepper_Left_CSN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : EDGE_LEFT_FRONT_INTERRUPT_Pin */
	GPIO_InitStruct.Pin = EDGE_LEFT_FRONT_INTERRUPT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(EDGE_LEFT_FRONT_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LASER_RIGHT_REAR_INTERRUPT_Pin LASER_REAR_LEFT_INTERRUPT_Pin */
	GPIO_InitStruct.Pin = LASER_RIGHT_REAR_INTERRUPT_Pin|LASER_REAR_LEFT_INTERRUPT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LASER_RIGHT_REAR_SHDN_Pin LASER_RIGHT_FRONT_SHDN_Pin LASER_LEFT_REAR_SHDN_Pin LASER_LEFT_FRONT_SHDN_Pin */
	GPIO_InitStruct.Pin = LASER_RIGHT_REAR_SHDN_Pin|LASER_RIGHT_FRONT_SHDN_Pin|LASER_LEFT_REAR_SHDN_Pin|LASER_LEFT_FRONT_SHDN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LASER_RIGHT_FRONT_INTERRUPT_Pin LASER_LEFT_REAR_INTERRUPT_Pin LASER_LEFT_FRONT_INTERRUPT_Pin LASER_REAR_RIGHT_INTERRUPT_Pin */
	GPIO_InitStruct.Pin = LASER_RIGHT_FRONT_INTERRUPT_Pin|LASER_LEFT_REAR_INTERRUPT_Pin|LASER_LEFT_FRONT_INTERRUPT_Pin|LASER_REAR_RIGHT_INTERRUPT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LASER_REAR_RIGHT_SHDN_Pin LASER_REAR_LEFT_SHDN_Pin */
	GPIO_InitStruct.Pin = LASER_REAR_RIGHT_SHDN_Pin|LASER_REAR_LEFT_SHDN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LASER_FRONT_RIGHT_INTERRUPT_Pin */
	GPIO_InitStruct.Pin = LASER_FRONT_RIGHT_INTERRUPT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(LASER_FRONT_RIGHT_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Stepper_Right_CSN_Pin */
	GPIO_InitStruct.Pin = Stepper_Right_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Stepper_Right_CSN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LASER_FRONT_RIGHT_SHDN_Pin LASER_FRONT_LEFT_SHDN_Pin */
	GPIO_InitStruct.Pin = LASER_FRONT_RIGHT_SHDN_Pin|LASER_FRONT_LEFT_SHDN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LASER_FRONT_LEFT_INTERRUPT_Pin */
	GPIO_InitStruct.Pin = LASER_FRONT_LEFT_INTERRUPT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
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




/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

	/* USER CODE BEGIN 5 */

	static char buffer[256];
	/* Infinite loop */
	for(;;)
	{
		//Bit 0:Edge left front Bit 1:Edge left rear Bit 2:Edge right front Bit 3: Edge right rear

		if(state_edges != 0x0){
			if(state_edges==0x1){
				//0001 Edge left front only triggered, reverse left side 100%, forward right side 50%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0060000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0030000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0x2){
				//0010 Edge left rear only triggered, forward left side 100%, reverse right side 50%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0060000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0030000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0x4){
				//0100 Edge right front only triggered, reverse right side 100%, forward left side 50%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0030000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0060000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0x8){
				//1000 Edge right rear only triggered, forward right side 100%, reverse left side 50%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0030000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0060000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0x3){
				//0011 Both left edges triggered, forward left side 100%, right side reverse 25%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0060000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0018000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0xC){
				//1100 Both right edges triggered, forward right 100%, left side reverse 25%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0018000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0060000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0x5){
				//0101 Both front edges triggered, reverse both 100%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0060000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0060000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else if(state_edges==0xA){
				//1010 Both rear edges triggered, forward both 100%
				xSemaphoreTake(SPIMutexHandle,100);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0060000);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0060000);
				xSemaphoreGive(SPIMutexHandle);
			}
			else{
				//Either the robot has imploded or physics is officially broken...
			}
		}

		else if(state_start_fight){
			//drive to center -> 	L +100% R -100%
			xSemaphoreTake(SPIMutexHandle,100);
			tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
			tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0000000);
			tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
			tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0000000);
			xSemaphoreGive(SPIMutexHandle);
			//once distance is equivalent to center, switch to tracking mode
			state_start_fight=0;
			state_edges=0x0;
		}

		else if(state_tracking){
			//side sensors -> turn towards tracking sensor, keep turning until front sensors detect.
			//front sensors -> 		L +100%	R +100%
			//rear sensors ->		L -100% R -100%

		}

		else if(state_seeking){
			//slowly spin -> 		L-25%	R +25%

		}

		else{
			//should not reach this state, reset all other states except start fight
			xSemaphoreTake(SPIMutexHandle,100);
			tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
			tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0x0000000);
			tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
			tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0x0000000);
			xSemaphoreGive(SPIMutexHandle);
		}


		//transmit the status to the dashboard
		if(DASHBOARD_CONNECTED){
			sprintf(buffer,
					"{'LFL':'%d','LFR':'%d','LRL':'%d','LRR':'%d','LLF':'%d','LLR':'%d','LRF':'%d','LRRI':'%d', 'EDG':'%i', LSM':'%d','RSM':'%d'}\r\n",
					RangingDataFrontLeft.RangeMilliMeter,
					RangingDataFrontRight.RangeMilliMeter,
					RangingDataRearLeft.RangeMilliMeter,
					RangingDataRearRight.RangeMilliMeter,
					RangingDataLeftFront.RangeMilliMeter,
					RangingDataLeftRear.RangeMilliMeter,
					RangingDataRightFront.RangeMilliMeter,
					RangingDataRightRear.RangeMilliMeter,
					state_edges,
					10,
					10);

			HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);
		}

		osDelay(IDLE_LOOP_DELAY);
	}
	/* USER CODE END 5 */
}

/* tskLASFRONTLEFT_fnc function */
void tskLASFRONTLEFT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASFRONTLEFT_fnc */
	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(I2CMutexHandle,100);
		VL53L1_GetMeasurementDataReady(LASER_FRONT_LEFT_DEV,&RangingDataFrontLeft.RangeStatus);
		if(RangingDataFrontLeft.RangeStatus==1){
			VL53L1_GetRangingMeasurementData(LASER_FRONT_LEFT_DEV,	&RangingDataFrontLeft);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_FRONT_LEFT_DEV);
		}
		xSemaphoreGive(I2CMutexHandle);
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASFRONTLEFT_fnc */
}

/* tskLASREARLEFT_fnc function */
void tskLASREARLEFT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASREARLEFT_fnc */
	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(I2CMutexHandle,100);
		VL53L1_GetMeasurementDataReady(LASER_REAR_LEFT_DEV,&RangingDataRearLeft.RangeStatus);
		if(RangingDataRearLeft.RangeStatus==1){
			VL53L1_GetRangingMeasurementData(LASER_REAR_LEFT_DEV,	&RangingDataRearLeft);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_REAR_LEFT_DEV);
		}
		xSemaphoreGive(I2CMutexHandle);
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASREARLEFT_fnc */
}

/* tskLASFRONTRIGHT_fnc function */
void tskLASFRONTRIGHT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASFRONTRIGHT_fnc */
	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(I2CMutexHandle,100);
		VL53L1_GetMeasurementDataReady(LASER_FRONT_RIGHT_DEV,&RangingDataFrontRight.RangeStatus);
		if(RangingDataFrontRight.RangeStatus==1){
			VL53L1_GetRangingMeasurementData(LASER_FRONT_RIGHT_DEV,&RangingDataFrontRight);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_FRONT_RIGHT_DEV);
		}
		xSemaphoreGive(I2CMutexHandle);
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASFRONTRIGHT_fnc */
}

/* tskLASREARRIGHT_fnc function */
void tskLASREARRIGHT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASREARRIGHT_fnc */
	/* Infinite loop */
	for(;;)
	{

		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASREARRIGHT_fnc */
}

/* tskLASLEFTFRONT_fnc function */
void tskLASLEFTFRONT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASLEFTFRONT_fnc */
	/* Infinite loop */
	for(;;)
	{
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASLEFTFRONT_fnc */
}

/* tskLASLEFTREAR_fnc function */
void tskLASLEFTREAR_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASLEFTREAR_fnc */
	/* Infinite loop */
	for(;;)
	{
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASLEFTREAR_fnc */
}

/* tskLASRIGHTFRONT_fnc function */
void tskLASRIGHTFRONT_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASRIGHTFRONT_fnc */
	/* Infinite loop */
	for(;;)
	{
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASRIGHTFRONT_fnc */
}

/* tskLASRIGHTREAR_fnc function */
void tskLASRIGHTREAR_fnc(void const * argument)
{
	/* USER CODE BEGIN tskLASRIGHTREAR_fnc */
	/* Infinite loop */
	for(;;)
	{
		osDelay(LASER_DELAY);
	}
	/* USER CODE END tskLASRIGHTREAR_fnc */
}

/* tskEDGE_fnc function */
void tskEDGE_fnc(void const * argument)
{
	/* USER CODE BEGIN tskEDGE_fnc */
	/* Infinite loop */
	for(;;)
	{
//		state_edges = 0x0;

		if(HAL_GPIO_ReadPin(EDGE_LEFT_FRONT_INTERRUPT_GPIO_Port, EDGE_LEFT_FRONT_INTERRUPT_Pin)==1){
			state_edges |= 0x1;
		}
		else if(HAL_GPIO_ReadPin(EDGE_LEFT_FRONT_INTERRUPT_GPIO_Port, EDGE_LEFT_FRONT_INTERRUPT_Pin)==0){
			state_edges &= ~0x1;
		}

		if(HAL_GPIO_ReadPin(EDGE_LEFT_REAR_INTERRUPT_GPIO_Port, EDGE_LEFT_REAR_INTERRUPT_Pin)==1){
			state_edges |= 0x2;
		}
		else if(HAL_GPIO_ReadPin(EDGE_LEFT_REAR_INTERRUPT_GPIO_Port, EDGE_LEFT_REAR_INTERRUPT_Pin)==0){
			state_edges &= ~0x2;
		}

		if(HAL_GPIO_ReadPin(EDGE_RIGHT_FRONT_INTERRUPT_GPIO_Port, EDGE_RIGHT_FRONT_INTERRUPT_Pin)==1){
			state_edges |= 0x4;
		}
		else if(HAL_GPIO_ReadPin(EDGE_RIGHT_FRONT_INTERRUPT_GPIO_Port, EDGE_RIGHT_FRONT_INTERRUPT_Pin)==0){
			state_edges &= ~0x4;
		}

		if(HAL_GPIO_ReadPin(EDGE_RIGHT_REAR_INTERRUPT_GPIO_Port, EDGE_RIGHT_REAR_INTERRUPT_Pin)==1){
			state_edges |= 0x8;
		}
		else if(HAL_GPIO_ReadPin(EDGE_RIGHT_REAR_INTERRUPT_GPIO_Port, EDGE_RIGHT_REAR_INTERRUPT_Pin)==0){
			state_edges &= ~0x8;
		}

		vTaskSuspend(NULL);
	}
	/* USER CODE END tskEDGE_fnc */
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
