/*
 * motorControl.h
 *
 *  Created on: Apr 28, 2024
 *      Author: Jeremy
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#define MOTOR_SPEED_MIN			50
#define MOTOR_SPEED_MAX			100

extern enum {
	frontRightMotor,
	frontLeftMotor,
	rearRightMotor,
	rearLeftMotor
} motor;

typedef struct {
	// timer handle for the timer containing the motor PWM channels
	TIM_HandleTypeDef motorTimerHandle;

	// store timer channel macros for each channel
	uint32_t frontRightPwmChannel;
	uint32_t frontLeftPwmChannel;
	uint32_t rearRightPwmChannel;
	uint32_t rearLeftPwmChannel;

	// current speed of each motor
	uint8_t frontRightSpeed;
	uint8_t frontLeftSpeed;
	uint8_t rearRightSpeed;
	uint8_t rearLeftSpeed;
} motorController;

void motorControllerInit(motorController* motrController, TIM_HandleTypeDef timerHandle);
void motorSetSpeed(motorController* motrController, uint32_t selectedMotor, uint8_t speed);
void setAllMotors(motorController* motrController, uint8_t speed);

#endif /* INC_MOTORCONTROL_H_ */
