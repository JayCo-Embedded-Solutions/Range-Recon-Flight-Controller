/*
 * flightController.h
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#ifndef INC_FLIGHTCONTROLLER_H_
#define INC_FLIGHTCONTROLLER_H_

#include "mpu6500.h"
#include "firFilter.h"
#include "motorControl.h"
#include "stdio.h"
#include "string.h"
#include "pidController.h"
#include "math.h"
#include "stm32f4xx_hal.h"

#define RAD2DEG 		57.3
#define GYRO_Prescale 	100

/* PID CONTROLLER VALUES */
#define RATE_PITCH_KP		0.03
#define RATE_PITCH_KI		0.0
#define RATE_PITCH_KD		0.0

#define RATE_ROLL_KP		0.03
#define RATE_ROLL_KI		0.0
#define RATE_ROLL_KD		0.0

#define RATE_YAW_KP			0.03
#define RATE_YAW_KI			0.0
#define RATE_YAW_KD			0.0


#define ANGLE_PITCH_KP		5
#define ANGLE_PITCH_KI		0.0
#define ANGLE_PITCH_KD		0.0

#define ANGLE_ROLL_KP		5
#define ANGLE_ROLL_KI		0.0
#define ANGLE_ROLL_KD		0.0

// accelerometer FIR filters
extern firFilter accelXDataLPF;
extern firFilter accelYDataLPF;
extern firFilter accelZDataLPF;

// rate mode and angle mode PID controllers
extern pidController rateModePitchController;
extern pidController rateModeRollController;
extern pidController rateModeYawController;

extern pidController angleModePitchController;
extern pidController angleModeRollController;
extern pidController angleModeYawController;

extern uint16_t gyroLastUpdate;

void flightControllerInit();
void imuExtractAndFilter(float* accelerometerData, float* gyroscopeData);
void updateCraftAngles(float* acceleromterData, float* gyroscopeData, float* aircraftAngles);
void rateController(float* aircraftAngularRates, float* desiredAngularRates, int16_t* controlSignals);
void angleController(float* aircraftAngles, float* desiredAngles, float* aircraftAngularRates, float* desiredAngularRates, int16_t* controlSignals);
void actuateMotors(uint8_t* currentMotorThrottle,uint8_t rcThrottle, int16_t* controlSignals);
uint8_t mapPWM(uint16_t joystickVal);

#endif /* INC_FLIGHTCONTROLLER_H_ */
