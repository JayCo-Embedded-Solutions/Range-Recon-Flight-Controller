/*
 * motorControl.c
 *
 *  Created on: Apr 28, 2024
 *      Author: jerem
 */

#include "stm32f3xx_hal.h"
#include "motorControl.h"
#include "stdio.h"
#include "string.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;

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

void initializeMotors(void) {
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

	HAL_Delay(1000);
}

void setAllMotors(uint8_t speed) {
	if(speed < PULSE_MIN || speed > PULSE_MAX) {}
	else {
		motorSetSpeed(frontRightMotor, speed);
		motorSetSpeed(frontLeftMotor, speed);
		motorSetSpeed(rearRightMotor, speed);
		motorSetSpeed(rearLeftMotor, speed);
	}
}

void testIncrementSpeed(void) {
	setAllMotors(50);
	HAL_Delay(4000);
	setAllMotors(60);
	HAL_Delay(4000);
	setAllMotors(70);
	HAL_Delay(4000);
	setAllMotors(80);
	HAL_Delay(4000);
}

void testIndividualAddress(void) {
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

	char myBuf[100];

	motorSetSpeed(frontRightMotor,60);
	sprintf(myBuf, "front right set to 60 \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t*)myBuf, strlen(myBuf), 100);
	HAL_Delay(3000);

	motorSetSpeed(frontLeftMotor,60);
	sprintf(myBuf, "front left set to 60 \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t*)myBuf, strlen(myBuf), 100);
	HAL_Delay(3000);

	motorSetSpeed(rearRightMotor,60);
	sprintf(myBuf, "rear right set to 60 \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t*)myBuf, strlen(myBuf), 100);
	HAL_Delay(3000);

	motorSetSpeed(rearLeftMotor,60);
	sprintf(myBuf, "rear left set to 60 \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t*)myBuf, strlen(myBuf), 100);
	HAL_Delay(3000);

	setAllMotors(PULSE_MIN);
	sprintf(myBuf, "all motors off \r\n");
	HAL_UART_Transmit(&huart4, (uint8_t*)myBuf, strlen(myBuf), 100);
	HAL_Delay(10000);
}
