/*
 * sumobot_functions.h
 *
 *  Created on: 3Aug.,2018
 *      Author: cdk
 */

#ifndef SUMOBOT_FUNCTIONS_H_
#define SUMOBOT_FUNCTIONS_H_

#include "stm32f4xx_hal.h"

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

}sEncoder;

#endif /* SUMOBOT_FUNCTIONS_H_ */
