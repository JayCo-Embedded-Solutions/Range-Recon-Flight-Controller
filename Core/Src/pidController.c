/*
 * pidController.c
 *
 *  Created on: May 15, 2024
 *      Author: jerem
 */

#include "pidController.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;

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
	controller->proportional = 0.0f;
	controller->integral = 0.0f;
	controller->derivative = 0.0f;

	controller->currentError = 0.0f;
	controller->prevError = 0.0f;
	controller->output = 0.0f;

	controller->dt = 0.0f;

	// save current time
	controller->lastUpdated = __HAL_TIM_GET_COUNTER(&htim2);
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
	// calculate current error
	controller->currentError = inputVal - desiredVal;

	// determine change in time since last sample
	controller->dt = (float)(__HAL_TIM_GET_COUNTER(&htim2) - controller->lastUpdated) / USecs2Secs;

	// calculate output value based on the gains and error
	controller->proportional = controller->KP * controller->currentError;
	controller->integral += controller->KI * controller->currentError * controller->dt;
	controller->derivative = controller->KD * ((controller->currentError - controller->prevError) / controller->dt);

	controller->output = controller->proportional + controller->integral + controller->derivative;

	// store current error in previous error and update sample time
	controller->prevError = controller->currentError;
	controller->lastUpdated = __HAL_TIM_GET_COUNTER(&htim2);

	return controller->output;
}
