/*
 * pidController.c
 *
 *  Created on: May 15, 2024
 *      Author: jerem
 */

#include "pidController.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim14;

void pidControllerInit(pidController* controller, float setKP, float setKI, float setKD) {
	controller->KP = setKP;
	controller->KI = setKI;
	controller->KD = setKD;

	controller->currentError = 0.0f;
	controller->prevError = 0.0f;
	controller->totalError = 0.0f;
	controller->output = 0.0f;

	controller->dt = 0.0f;
	controller->lastUpdated = __HAL_TIM_GET_COUNTER(&htim14);
}

float pidUpdateOutput(pidController* controller, float inputVal, float desiredVal) {
	controller->currentError = inputVal - desiredVal;
	controller->totalError += controller->currentError;

	controller->dt = (__HAL_TIM_GET_COUNTER(&htim14) - controller->lastUpdated) / USecs2Secs;

	controller->output = (controller->KP * controller->currentError) +
						 (controller->KI * controller->totalError * controller->dt) +
						 (controller->KD * (controller->currentError - controller->prevError) / controller->dt);

	controller->prevError = controller->currentError;
	controller->lastUpdated = __HAL_TIM_GET_COUNTER(&htim14);

	return controller->output;
}
