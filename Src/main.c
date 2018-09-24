
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "vl53l1_api.h"
#include "string.h"
#include "sumobot_functions.h"
#include "TMC5160.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
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

VL53L1_DetectionConfig_t Detectionconfig;

//State machine states
static unsigned char robot_state=STATE_IDLE;
static unsigned char previous_state=STATE_IDLE;

unsigned char state_edges= 0x0; //Bit 0:Edge left front Bit 1:Edge left rear Bit 2:Edge right front Bit 3: Edge right rear

static int64_t ScaledMotorMagnitude = 0;
static int8_t ScaledMotorDirection = 0;
float Opponent_Theta =0;
static char buffer[256];
char timer_block = 1;

//I2C bus variables
uint8_t measurementDataReady = 0x0;

//SPI bus variables
uint8_t spiTXbuffer[5], spiRXbuffer[5];
uint8_t SPISTATUS;

//Motors
sTMC5160Motor left_stepper;
sTMC5160Motor right_stepper;

uint32_t StartTime = 0;
uint32_t CurrentTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);

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
	//Setup various configs
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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


	//Initialise the distance sensors
	SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS,LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_FRONT_RIGHT_DEV,LASFRONTRIGHT_I2C_ADDRESS, LASER_FRONT_RIGHT_SHDN_GPIO_Port,LASER_FRONT_RIGHT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_REAR_LEFT_DEV, LASREARLEFT_I2C_ADDRESS,LASER_REAR_LEFT_SHDN_GPIO_Port, LASER_REAR_LEFT_SHDN_Pin);
	SetupVL53L1XDevices(LASER_REAR_RIGHT_DEV, LASREARRIGHT_I2C_ADDRESS, LASER_REAR_RIGHT_SHDN_GPIO_Port, LASER_REAR_RIGHT_SHDN_Pin);

	//Initialise the motors
	tmc5160_initialise_motor(left_stepper);
	tmc5160_initialise_motor(right_stepper);


	state_edges= 0x0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


		HAL_IWDG_Refresh(&hiwdg);

		//-----------------Lasers-----------------
		//Front left
		VL53L1_GetMeasurementDataReady(LASER_FRONT_LEFT_DEV,&measurementDataReady);
		if(measurementDataReady == 0x01){
			VL53L1_GetRangingMeasurementData(LASER_FRONT_LEFT_DEV,	&RangingDataFrontLeft);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_FRONT_LEFT_DEV);
			measurementDataReady = 0x0;
		}

		//Front right
		VL53L1_GetMeasurementDataReady(LASER_FRONT_RIGHT_DEV,&measurementDataReady);
		if(measurementDataReady == 0x01){
			VL53L1_GetRangingMeasurementData(LASER_FRONT_RIGHT_DEV,	&RangingDataFrontRight);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_FRONT_RIGHT_DEV);
			measurementDataReady = 0x0;
		}

		//Rear left
		VL53L1_GetMeasurementDataReady(LASER_REAR_LEFT_DEV,&measurementDataReady);
		if(measurementDataReady == 0x01){
			VL53L1_GetRangingMeasurementData(LASER_REAR_LEFT_DEV,	&RangingDataRearLeft);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_REAR_LEFT_DEV);
			measurementDataReady = 0x0;
		}

		//Rear right
		VL53L1_GetMeasurementDataReady(LASER_REAR_RIGHT_DEV,&measurementDataReady);
		if(measurementDataReady == 0x01){
			VL53L1_GetRangingMeasurementData(LASER_REAR_RIGHT_DEV,	&RangingDataRearRight);
			VL53L1_ClearInterruptAndStartMeasurement(LASER_REAR_RIGHT_DEV);
			measurementDataReady = 0x0;
		}


		if(		RangingDataFrontLeft.RangeMilliMeter < 0 ||	RangingDataFrontRight.RangeMilliMeter < 0 ||
				RangingDataRearLeft.RangeMilliMeter < 0 || RangingDataRearRight.RangeMilliMeter < 0){

			SetupVL53L1XDevices(LASER_FRONT_LEFT_DEV, LASFRONTLEFT_I2C_ADDRESS,LASER_FRONT_LEFT_SHDN_GPIO_Port, LASER_FRONT_LEFT_SHDN_Pin);
			SetupVL53L1XDevices(LASER_FRONT_RIGHT_DEV,LASFRONTRIGHT_I2C_ADDRESS, LASER_FRONT_RIGHT_SHDN_GPIO_Port,LASER_FRONT_RIGHT_SHDN_Pin);
			SetupVL53L1XDevices(LASER_REAR_LEFT_DEV, LASREARLEFT_I2C_ADDRESS,LASER_REAR_LEFT_SHDN_GPIO_Port, LASER_REAR_LEFT_SHDN_Pin);
			SetupVL53L1XDevices(LASER_REAR_RIGHT_DEV, LASREARRIGHT_I2C_ADDRESS, LASER_REAR_RIGHT_SHDN_GPIO_Port, LASER_REAR_RIGHT_SHDN_Pin);
		}


		//------------TRACKING LOGIC---------
		if (state_edges != 0x0 && EDGES_ENABLED == 1){
			robot_state = STATE_EDGE_RETREAT;
		}
		else if(RangingDataFrontLeft.RangeMilliMeter < MINIMUM_SENSE_DISTANCE && RangingDataFrontRight.RangeMilliMeter< MINIMUM_SENSE_DISTANCE){
			ScaledMotorMagnitude = ATTACK_SPEED;
			ScaledMotorDirection = DIR_FORWARD;
			robot_state = STATE_SEEKING;
		}
		else if(RangingDataFrontRight.RangeMilliMeter < MINIMUM_SENSE_DISTANCE){
			ScaledMotorMagnitude = SEEK_SPEED;
			ScaledMotorDirection = DIR_RIGHT;
			robot_state = STATE_SEEKING;
		}
		else if(RangingDataFrontLeft.RangeMilliMeter < MINIMUM_SENSE_DISTANCE){
			ScaledMotorMagnitude = SEEK_SPEED;
			ScaledMotorDirection = DIR_LEFT;
			robot_state = STATE_SEEKING;
		}
		else if(RangingDataRearLeft.RangeMilliMeter < MINIMUM_SENSE_DISTANCE && RangingDataRearRight.RangeMilliMeter< MINIMUM_SENSE_DISTANCE){
			ScaledMotorMagnitude = ATTACK_SPEED;
			ScaledMotorDirection = DIR_REVERSE;
			robot_state = STATE_SEEKING;
		}
		else if(RangingDataRearLeft.RangeMilliMeter < MINIMUM_SENSE_DISTANCE){
			ScaledMotorMagnitude = SEEK_SPEED;
			ScaledMotorDirection = DIR_RIGHT;
			robot_state = STATE_SEEKING;
		}
		else if(RangingDataRearRight.RangeMilliMeter < MINIMUM_SENSE_DISTANCE){
			ScaledMotorMagnitude = SEEK_SPEED;
			ScaledMotorDirection = DIR_LEFT;
			robot_state = STATE_SEEKING;
		}
		else{
			ScaledMotorMagnitude = 0;
			ScaledMotorDirection = DIR_FORWARD;
			robot_state = STATE_IDLE;
		}

		if(timer_block){
			while(timer_block){
				HAL_Delay(10);
			}
			tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0);
			tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0);

			StartTime = HAL_GetTick();
			CurrentTime = HAL_GetTick();

			while((CurrentTime-StartTime)<5000){

				CurrentTime = HAL_GetTick();
				sprintf(buffer,
						"{'Time':'%d'}\r\n",
						(CurrentTime-StartTime));

				HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);

				HAL_IWDG_Refresh(&hiwdg);

			}
		}



		//-------------STATES--------------
		switch(robot_state){
		case STATE_EDGE_RETREAT:

			switch(state_edges){
			case 0x1:
				//0001 Edge left front only triggered, reverse left side 100%, forward right side 50%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_HALF_SPEED);
				break;

			case 0x2:
				//0010 Edge left rear only triggered, forward left side 100%, reverse right side 50%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_HALF_SPEED);
				break;

			case 0x4:
				//0100 Edge right front only triggered, reverse right side 100%, forward left side 50%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_HALF_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);
				break;

			case 0x8:
				//1000 Edge right rear only triggered, forward right side 100%, reverse left side 50%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_HALF_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);
				break;

			case 0x3:
				//0011 Both left edges triggered, forward left side 100%, right side reverse 25%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_QUARTER_SPEED);
				break;

			case 0xC:
				//1100 Both right edges triggered, forward right 100%, left side reverse 25%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_QUARTER_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);
				break;

			case 0x5:
				//0101 Both front edges triggered, reverse both 100%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);
				break;

			case 0xA:
				//1010 Both rear edges triggered, forward both 100%
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);

				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, EDGE_RETREAT_MAX_SPEED);
				break;
			}
			break;

			case STATE_SEEKING:
				//drive to center -> 	L +100% R -100%

				switch(ScaledMotorDirection){
				case DIR_LEFT:
					tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
					tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
					break;

				case DIR_RIGHT:
					tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
					tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
					break;

				case DIR_FORWARD:
					tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
					tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
					break;

				case DIR_REVERSE:
					tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
					tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);
					break;

				}

				tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, ScaledMotorMagnitude);
				tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, ScaledMotorMagnitude);

//				previous_state=STATE_SEEKING;
				break;

				case STATE_ATTACKING:
//					previous_state=STATE_ATTACKING;
					break;

				case STATE_IDLE:
//					previous_state=STATE_IDLE;
					tmc5160_writeInt(Stepper_Left_CSN_GPIO_Port, Stepper_Left_CSN_Pin,TMC5160_VMAX, 0);
					tmc5160_writeInt(Stepper_Right_CSN_GPIO_Port, Stepper_Right_CSN_Pin,TMC5160_VMAX, 0);

					break;


		}

		if(DASHBOARD_CONNECTED){
			sprintf(buffer,
					"{'LFL':'%d','LFR':'%d','LRL':'%d','LRR':'%d', 'MOTOR':'%d'}\r\n",
					RangingDataFrontLeft.RangeMilliMeter,
					RangingDataFrontRight.RangeMilliMeter,
					RangingDataRearLeft.RangeMilliMeter,
					RangingDataRearRight.RangeMilliMeter,
					ScaledMotorMagnitude);

			HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 500);
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOC, Stepper_Left_CSN_Pin|LASER_REAR_RIGHT_SHDN_Pin|LASER_FRONT_LEFT_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LASER_REAR_LEFT_SHDN_Pin|Stepper_Right_CSN_Pin|LASER_FRONT_RIGHT_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : LASER_REAR_LEFT_SHDN_Pin LASER_FRONT_RIGHT_SHDN_Pin */
  GPIO_InitStruct.Pin = LASER_REAR_LEFT_SHDN_Pin|LASER_FRONT_RIGHT_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EDGE_LEFT_FRONT_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = EDGE_LEFT_FRONT_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EDGE_LEFT_FRONT_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_REAR_RIGHT_SHDN_Pin LASER_FRONT_LEFT_SHDN_Pin */
  GPIO_InitStruct.Pin = LASER_REAR_RIGHT_SHDN_Pin|LASER_FRONT_LEFT_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Stepper_Right_CSN_Pin */
  GPIO_InitStruct.Pin = Stepper_Right_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Stepper_Right_CSN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

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
