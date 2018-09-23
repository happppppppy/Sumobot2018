/*
 * sumobot_functions.h
 *
 *  Created on: 3Aug.,2018
 *      Author: cdk
 */

#ifndef SUMOBOT_FUNCTIONS_H_
#define SUMOBOT_FUNCTIONS_H_

#include "stm32f4xx_hal.h"
#include "TMC5160.h"
#include "vl53l1_api.h"

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern VL53L1_DetectionConfig_t Detectionconfig;
extern unsigned char state_edges;

typedef struct {
	int currentValue;
	int setPoint;
	int PIDMax;
	int PIDMin;
	float Kp;
	float Ki;
	float Kd;
	float derivator;
	float integrator;
	int integratorMax;
	int integratorMin;
	int outputValue;
} sPIDController;

typedef sPIDController *sPIDController_h;

typedef struct {
	long encCountPre;
	long encCountPost;
	unsigned long timTickPre;
	unsigned long timTickPost;
	unsigned long speed;
	uint8_t countPerRevolution;

} sEncoder;

typedef struct {
	uint32_t velocity_start;
	uint32_t acceleration_1;
	uint32_t velocity_1;
	uint32_t acceleration_max;
	uint32_t velocity_max;
	uint32_t deceleration_max;
	uint32_t deceleration_1;
	uint32_t velocity_stop;
	uint16_t gpio_pin;
	GPIO_TypeDef* gpio_port;
	unsigned char direction;
} sTMC5160Motor;

typedef sTMC5160Motor *sTMC5160Motor_h;

void tmc5160_writeInt(GPIO_TypeDef* CSN_GPIO_Port, uint16_t CSN_GPIO_Pin, uint8_t address, int value);
void tmc40bit_writeInt(GPIO_TypeDef* CSN_GPIO_Port,uint16_t CSN_GPIO_Pin, uint8_t address, int value);
void tmc5160_initialise_motor(sTMC5160Motor Motor);
void PIDController(sPIDController *sPIDController);
void EncoderController(sEncoder *sEncoder, TIM_HandleTypeDef *htim2);
VL53L1_Error SetupVL53L1XDevices(VL53L1_DEV Device_DEV, int Device_ADDRESS,
		GPIO_TypeDef* Device_SHDNPORT, uint16_t Device_SHDNPIN);
float degreesToRadians(float angleDegrees);
float radiansToDegrees(float angleRadians);
void updateEdgeState();
//------------------------------------------------------------------------------------------------------


//DEFINES

//RTOS constants
#define LASER_DELAY 80
#define IDLE_LOOP_DELAY 10

//Defines for laser sensors
#define LASLEFTFRONT_I2C_ADDRESS 0x10
#define LASLEFTREAR_I2C_ADDRESS 0x20
#define LASRIGHTFRONT_I2C_ADDRESS 0x30
#define LASRIGHTREAR_I2C_ADDRESS 0x40
#define LASFRONTLEFT_I2C_ADDRESS 0x50
#define LASFRONTRIGHT_I2C_ADDRESS 0x60
#define LASREARLEFT_I2C_ADDRESS 0x70
#define LASREARRIGHT_I2C_ADDRESS 0x80

#define LASER_SENSOR_TIMING_BUDGET_US 60000
#define LASER_SENSOR_MEASUREMENT_PERIOD_MS 70

#define LASER_SENSOR_HIGH_THRESHOLD 1000
#define LASER_SENSOR_LOW_THRESHOLD 0

#define FRONT_LEFT_LASER_X_OFFSET -10.0 //Note, 0,0 is the centre of the robot
#define FRONT_LEFT_LASER_Y_OFFSET 106.0
#define FRONT_LEFT_LASER_THETA_OFFSET 345.0

#define FRONT_RIGHT_LASER_X_OFFSET 10.0
#define FRONT_RIGHT_LASER_Y_OFFSET 106.0
#define FRONT_RIGHT_LASER_THETA_OFFSET 15.0

#define REAR_LEFT_LASER_X_OFFSET -10.0
#define REAR_LEFT_LASER_Y_OFFSET -106.0
#define REAR_LEFT_LASER_THETA_OFFSET 195.0

#define REAR_RIGHT_LASER_X_OFFSET 10.0
#define REAR_RIGHT_LASER_Y_OFFSET -106.0
#define REAR_RIGHT_LASER_THETA_OFFSET 165.0

#define OPPONENT_POSITION_GAIN 10.0

#define M_PI 3.1416
//#define radiansToDegrees(angleRadians) (180.0 * angleRadians / M_PI)

//Defines for edge sensors


//Defines for TMC5160
#define LM_VSTART 1
#define LM_A1 20000
#define LM_V1 0
#define LM_AMAX 20000
#define LM_VMAX 0
#define LM_DMAX 20000
#define LM_D1 20000
#define LM_VSTOP 10
#define LM_DIRECTION 0

#define RM_VSTART 1
#define RM_A1 20000
#define RM_V1 0
#define RM_AMAX 20000
#define RM_VMAX 0
#define RM_DMAX 20000
#define RM_D1 20000
#define RM_VSTOP 10
#define RM_DIRECTION 1

//Motor speeds
#define EDGE_RETREAT_MAX_SPEED 250000//0x003D090
#define EDGE_RETREAT_HALF_SPEED 125000//0x001E848
#define EDGE_RETREAT_QUARTER_SPEED 62500//0x000F424

#define SEEK_MAX 100000
#define SEEK_MIN 00000

#define SPEED_GAIN 40

#define PID_DEADBAND 500

//states
#define STATE_IDLE 0x0
#define STATE_5SECWAIT 0x1
#define STATE_SEEKING 0x2
#define STATE_TRACKING 0x3
#define STATE_ATTACKING 0x4
#define STATE_EDGE_RETREAT 0x5

//Distance sensor constants
#define MINIMUM_SENSE_DISTANCE 1300.0


//debug defines
#define DASHBOARD_CONNECTED 1
#define MOTORS_ENABLED 1
#define EDGES_ENABLED 0

#endif /* SUMOBOT_FUNCTIONS_H_ */
