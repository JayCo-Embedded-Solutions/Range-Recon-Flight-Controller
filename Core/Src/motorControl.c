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
 * @brief: initializes a motorController struct
 *
 * @param motrController: pointer to a motorController struct object
 * @param timerHandle: handle for the timer that feeds the motor PWM channels
 *
 * @returns: none
 */
void motorControllerInit(motorController* motrController, TIM_HandleTypeDef timerHandle) {
  // assign motor timer handle to the timer handle passed in
  motrController->motorTimerHandle = timerHandle;

  // assign pwm channels to struct variables
  motrController->frontRightPwmChannel = TIM_CHANNEL_1;
  motrController->frontLeftPwmChannel = TIM_CHANNEL_2;
  motrController->rearRightPwmChannel = TIM_CHANNEL_3;
  motrController->rearLeftPwmChannel = TIM_CHANNEL_4;

  // start pwm generation on motor channels
  HAL_TIM_PWM_Start(&motrController->motorTimerHandle, motrController->frontRightPwmChannel);
  HAL_TIM_PWM_Start(&motrController->motorTimerHandle, motrController->frontLeftPwmChannel);
  HAL_TIM_PWM_Start(&motrController->motorTimerHandle, motrController->rearRightPwmChannel);
  HAL_TIM_PWM_Start(&motrController->motorTimerHandle, motrController->rearLeftPwmChannel);

  // initialize all motors to zero speed
  motorSetSpeed(motrController, frontRightMotor, MOTOR_SPEED_MIN);
  motorSetSpeed(motrController, frontLeftMotor, MOTOR_SPEED_MIN);
  motorSetSpeed(motrController, rearRightMotor, MOTOR_SPEED_MIN);
  motorSetSpeed(motrController, rearLeftMotor, MOTOR_SPEED_MIN);
}

/**
 * @brief: sets the speed of an individual motor for a motorController object
 *
 * @param motrController: pointer to a motorController struct object
 * @param selectedMotor: the desired motor of which to change the speed. the motor names are enumerated in motorControl.h
 * @param speed: the desired speed to set the motor, ranges from 50 (1ms pulse) to 100 (2ms pulse)
 *
 * @returns: none
 */
void motorSetSpeed(motorController* motrController, uint32_t selectedMotor, uint8_t speed) {
  switch(selectedMotor) {
  case frontRightMotor:
	  motrController->frontRightSpeed = speed;
	  motrController->motorTimerHandle.Instance->CCR1 = motrController->frontRightSpeed;
	  break;
  case frontLeftMotor:
	  motrController->frontLeftSpeed = speed;
	  motrController->motorTimerHandle.Instance->CCR2 = motrController->frontLeftSpeed;
	  break;
  case rearRightMotor:
	  motrController->rearRightSpeed = speed;
	  motrController->motorTimerHandle.Instance->CCR3 = motrController->rearRightSpeed;
	  break;
  case rearLeftMotor:
	  motrController->rearLeftSpeed = speed;
	  motrController->motorTimerHandle.Instance->CCR4 = motrController->rearLeftSpeed;
	  break;
  }
}

/**
 * @brief: sets the speeds of all motors for a given motorController object
 *
 * @param motrController: pointer to a motorController struct object
 * @param speed: the desired speed at which to set all motors
 *
 * @returns: none
 */
void setAllMotors(motorController* motrController, uint8_t speed) {
  motorSetSpeed(motrController, frontRightMotor, speed);
  motorSetSpeed(motrController, frontLeftMotor, speed);
  motorSetSpeed(motrController, rearRightMotor, speed);
  motorSetSpeed(motrController, rearLeftMotor, speed);
}
