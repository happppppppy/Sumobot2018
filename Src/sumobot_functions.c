#include "sumobot_functions.h"

void PIDController(sPIDController *sPIDController);
void EncoderController(sEncoder *sEncoder, TIM_HandleTypeDef *htim2);

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

