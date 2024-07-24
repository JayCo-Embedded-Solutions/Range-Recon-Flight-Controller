/*
 * motorControl.c
 *
 *  Created on: Apr 28, 2024
 *      Author: jerem
 */

#include "stm32f4xx_hal.h"
#include "motorControl.h"
#include "stdio.h"
#include "string.h"

extern TIM_HandleTypeDef htim1;

/**
 * @brief: initializes the timer channels for the drone motors
 *
 * @returns: none
 */
void initializeMotors() {
	// begin PWM generation on all motor channels
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	// initialize all motors to zero speed so the ESCs can set the reference
	motorSetSpeed(frontRightMotor, PULSE_MIN);
	motorSetSpeed(frontLeftMotor, PULSE_MIN);
	motorSetSpeed(rearRightMotor, PULSE_MIN);
	motorSetSpeed(rearLeftMotor, PULSE_MIN);

	HAL_Delay(4000);
}

/**
 * @brief: sets the speed of an individual motor
 *
 * @param selectedMotor: the motor of which to set the speed
 * @param speed: the speed at which to set the selected motor
 *
 * @returns: none
 */
void motorSetSpeed(uint8_t selectedMotor, uint8_t speed) {
	switch(selectedMotor) {
		case frontRightMotor:
			htim1.Instance->CCR1 = speed;
			break;
		case frontLeftMotor:
			htim1.Instance->CCR2 = speed;
			break;
		case rearRightMotor:
			htim1.Instance->CCR3 = speed;
			break;
		case rearLeftMotor:
			htim1.Instance->CCR4 = speed;
			break;
	}
}

/**
 * @brief: sets all motors to the same speed
 *
 * @param speed: the speed at which to set all of the motors
 *
 * @returns: none
 */
void setAllMotors(uint8_t speed) {
	if(speed < PULSE_MIN || speed > PULSE_MAX) {}
	else {
		motorSetSpeed(frontRightMotor, speed);
		motorSetSpeed(frontLeftMotor, speed);
		motorSetSpeed(rearRightMotor, speed);
		motorSetSpeed(rearLeftMotor, speed);
	}
}

/**
 * @brief: tests incrementing the speed of all motors at the same time
 *
 * @returns: none
 */
void testIncrementSpeed() {
	setAllMotors(50);
	HAL_Delay(4000);
	setAllMotors(60);
	HAL_Delay(4000);
	setAllMotors(70);
	HAL_Delay(4000);
	setAllMotors(80);
	HAL_Delay(4000);
}

/**
 * @brief: tests the individual addressibility of the motors, start from the front right motor and ending with rear left
 *
 * @returns: none
 */
void testIndividualAddress() {
	motorSetSpeed(frontRightMotor, 55);
	HAL_Delay(2000);

	motorSetSpeed(frontRightMotor, PULSE_MIN);
	motorSetSpeed(frontLeftMotor, 55);
	HAL_Delay(2000);

	motorSetSpeed(frontLeftMotor, PULSE_MIN);
	motorSetSpeed(rearRightMotor, 55);
	HAL_Delay(2000);

	motorSetSpeed(rearRightMotor, PULSE_MIN);
	motorSetSpeed(rearLeftMotor, 55);
	HAL_Delay(2000);

	motorSetSpeed(rearLeftMotor, PULSE_MIN);
	HAL_Delay(5000);

	motorSetSpeed(frontRightMotor,60);
	HAL_Delay(3000);

	motorSetSpeed(frontLeftMotor,60);
	HAL_Delay(3000);

	motorSetSpeed(rearRightMotor,60);
	HAL_Delay(3000);

	motorSetSpeed(rearLeftMotor,60);
	HAL_Delay(3000);

	setAllMotors(PULSE_MIN);
	HAL_Delay(10000);
}
