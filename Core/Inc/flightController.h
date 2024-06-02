/*
 * flightController.h
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#ifndef INC_FLIGHTCONTROLLER_H_
#define INC_FLIGHTCONTROLLER_H_

#include "mpu6500.h"
#include "bmp390.h"
#include "firFilter.h"
#include "motorControl.h"
#include "stdio.h"
#include "string.h"
#include "pidController.h"
#include "math.h"
#include "stm32f4xx_hal.h"

#define RAD2DEG 		57.2957795131
#define GYRO_Prescale 	100

/* PID CONTROLLER VALUES */
#define RATE_PITCH_KP		0.05
#define RATE_PITCH_KI		0.00001
#define RATE_PITCH_KD		0.0

#define RATE_ROLL_KP		0.05
#define RATE_ROLL_KI		0.00001
#define RATE_ROLL_KD		0.0

#define RATE_YAW_KP			0.05
#define RATE_YAW_KI			0.00001
#define RATE_YAW_KD			0.0

#define ANGLE_PITCH_KP		5
#define ANGLE_PITCH_KI		0.0001
#define ANGLE_PITCH_KD		0.0

#define ANGLE_ROLL_KP		5
#define ANGLE_ROLL_KI		0.0001
#define ANGLE_ROLL_KD		0.0

#define VERTICAL_VELOCITY_KP  1
#define VERTICAL_VELOCITY_KI  0.0
#define VERTICAL_VELOCITY_KD  0.0

// rate mode and angle mode PID controllers
extern pidController rateModePitchController;
extern pidController rateModeRollController;
extern pidController rateModeYawController;

extern pidController angleModePitchController;
extern pidController angleModeRollController;

void flightControllerInit();
void rateController(MPU6500* mpu, float* desiredAngularRates, float* controlSignals);
void angleController(MPU6500* mpu, float* desiredAngles, float* desiredAngularRates, float* controlSignals);
void actuateMotors(float* currentMotorThrottle, float* controlSignals, float* vvAccumulator, float verticalVelocityMotorAdjustment);
void updateVerticalVelocityControl(float measuredVerticalVelocity, float desiredVerticalVelocity, float* verticalVelocityMotorAdjustment);
void getKalmanAltitude(MPU6500* mpu, BMP390* bmp, float bmpOffset, float* altitude, float* verticalVelocity, float* pVals, float timeDiff);
float mapRCRaw(uint16_t joystickVal);

#endif /* INC_FLIGHTCONTROLLER_H_ */
