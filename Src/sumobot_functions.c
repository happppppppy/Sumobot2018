#include "sumobot_functions.h"

void PIDController(sPIDController *sPIDController) {
	int error = sPIDController->setPoint - sPIDController->currentValue;
	int P_value = sPIDController->Kp * error;
	int D_value = sPIDController->Kd * (error - sPIDController->derivator);

	sPIDController->derivator = error;
	sPIDController->integrator = sPIDController->integrator + error;

	if (sPIDController->integrator > sPIDController->integratorMax) {
		sPIDController->integrator = sPIDController->integratorMax;
	} else if (sPIDController->integrator < sPIDController->integratorMin) {
		sPIDController->integrator = sPIDController->integratorMin;
	}

	int I_value = sPIDController->integrator * sPIDController->Ki;

	sPIDController->outputValue = P_value + I_value + D_value;

	if (sPIDController->outputValue > sPIDController->PIDMax) {
		sPIDController->outputValue = sPIDController->PIDMax;
	} else if (sPIDController->outputValue < sPIDController->PIDMin) {
		sPIDController->outputValue = sPIDController->PIDMin;
	}

	//startPulse = (pulseMin-pulseMax)*(PID_Output-PIDMin)/(PIDMax-PIDMin)+pulseMax;
}

void EncoderController(sEncoder *sEncoder, TIM_HandleTypeDef *htim2){
	sEncoder->encCountPre = htim2->Instance->CNT;
	sEncoder->timTickPre = HAL_GetTick();
	sEncoder->speed = (sEncoder->encCountPre - sEncoder->encCountPost)/2;
	sEncoder->encCountPost = sEncoder->encCountPre;
	sEncoder->timTickPost = sEncoder->timTickPre;
}

void tmc5160_writeInt(GPIO_TypeDef* CSN_GPIO_Port, uint16_t CSN_GPIO_Pin, uint8_t address, int value)
{
    tmc40bit_writeInt(CSN_GPIO_Port, CSN_GPIO_Pin, address, value);
}

void tmc40bit_writeInt(GPIO_TypeDef* CSN_GPIO_Port,uint16_t CSN_GPIO_Pin, uint8_t address, int value)
{
	uint8_t tbuf[5];
    tbuf[0] = address | 0x80;
    tbuf[1] = 0xFF & (value>>24);
    tbuf[2] = 0xFF & (value>>16);
    tbuf[3] = 0xFF & (value>>8);
    tbuf[4] = 0xFF & value;

	HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_GPIO_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,tbuf,5,50);
	HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_GPIO_Pin,GPIO_PIN_SET);

}

void tmc5160_initialise_motor(sTMC5160Motor Motor){
	////TMC5160 Startup SPI Code

	// MULTISTEP_FILT=1, EN_PWM_MODE=1 enables stealthChop™
	if(Motor.direction==0){
		tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_GCONF, 0x0000000C);
	}
	else{
		tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_GCONF, 0x0000001C);
	}

	// TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle™)
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_CHOPCONF, 0x000100C3);

	// IHOLD=10, IRUN=15 (max. current), IHOLDDELAY=6
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_IHOLD_IRUN, 0x00080F05);

	// TPOWERDOWN=10: Delay before power down in stand still
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_TPOWERDOWN, 0x0000000A);

	// TPWMTHRS=500
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_TPWMTHRS, 0x000001F4);

	// TPWMCONF - PWM_LIM=12, PWM_REG=4, freewheel1=0, freewheel0=0, pwm_autograd=1, pwm_autoscale=1,
	// PWM_GRAD=240, PWM_OFS=30
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_PWMCONF, 0x1EF0304C);

	// Values for speed and acceleration
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_VSTART, Motor.velocity_start);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_A1, Motor.acceleration_1);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_V1, Motor.velocity_1);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_AMAX, Motor.acceleration_max);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_VMAX, Motor.velocity_max);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_DMAX, Motor.deceleration_max);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_D1, Motor.deceleration_1);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_VSTOP, Motor.velocity_stop);
	tmc5160_writeInt(Motor.gpio_port, Motor.gpio_pin, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);

}

VL53L1_Error SetupVL53L1XDevices(VL53L1_DEV Device_DEV, int Device_ADDRESS,
		GPIO_TypeDef* Device_SHDNPORT, uint16_t Device_SHDNPIN) {
	VL53L1_Error status;
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
