/*
 * pidController.c
 *
 *  Created on: May 15, 2024
 *      Author: jerem
 */

#include "pidController.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim14;

/**
 * @brief: initializes a pidController object
 *
 * @param controller: pointer to a pidController struct
 * @param setKP: desired proportional constant for controller
 * @param setKI: desired integral constant for controller
 * @param setKD: desired derivative constant for controller
 *
 * @returns: none
 */
void pidControllerInit(pidController* controller, float setKP, float setKI, float setKD) {
	// initialize gain constants
	controller->KP = setKP;
	controller->KI = setKI;
	controller->KD = setKD;

	// initialize all other values to 0.0f
	controller->currentError = 0.0f;
	controller->prevError = 0.0f;
	controller->totalError = 0.0f;
	controller->output = 0.0f;

	controller->dt = 0.0f;

	// save current time
	controller->lastUpdated = __HAL_TIM_GET_COUNTER(&htim14);
}

/**
 * @brief: calculates and outputs control signal based on the input and desired signals
 *
 * @param controller: pointer to a pidController struct
 * @param inputVal: the actual value of a signal
 * @param desiredVal: the desired value of a signal
 *
 * @returns: controller output signal
 */
float pidUpdateOutput(pidController* controller, float inputVal, float desiredVal) {
	// calculate current error and add to the total error
	controller->currentError = inputVal - desiredVal;
	controller->totalError += controller->currentError;

	// determine change in time since last sample
	controller->dt = (__HAL_TIM_GET_COUNTER(&htim14) - controller->lastUpdated) / USecs2Secs;

	// calculate output value based on the gains and error
	controller->output = (controller->KP * controller->currentError) +
						 (controller->KI * controller->totalError * controller->dt) +
						 (controller->KD * (controller->currentError - controller->prevError) / controller->dt);

	// store current error in previous error and update sample time
	controller->prevError = controller->currentError;
	controller->lastUpdated = __HAL_TIM_GET_COUNTER(&htim14);

	return controller->output;
}
