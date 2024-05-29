/*
 * flightController.c
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#include "flightController.h"

pidController rateModePitchController;
pidController rateModeRollController;
pidController rateModeYawController;

pidController angleModePitchController;
pidController angleModeRollController;

/**
 * Initializes all necessary elements of the flight controller (gyro update time, accelerometer filters, PID controllers)
 */
void flightControllerInit() {
	// initialize PID controllers for the rate mode controller
	pidControllerInit(&rateModePitchController, RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
	pidControllerInit(&rateModeRollController, RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
	pidControllerInit(&rateModeYawController, RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD);

	// initialize PID controllers for the angle mode controller
	pidControllerInit(&angleModePitchController, ANGLE_PITCH_KP, ANGLE_PITCH_KI, ANGLE_PITCH_KD);
	pidControllerInit(&angleModeRollController, ANGLE_ROLL_KP, ANGLE_ROLL_KI, ANGLE_ROLL_KD);
}

/**
 * Determines required adjustments in the x, y, and z directions based on actual and desired angular rates of the aircraft
 *
 * @param aircraftAngularRates: pointer to a 3-element array containing the current angular acceleration rates in the x, y, z directions
 * @param desiredAngularRates: pointer to a 3-element array containing the desired angular acceleration rates in the x, y, z directions
 * @param controlSignals: pointer to a 3-element array containing the required compensation in the x, y, z directions
 */
void rateController(MPU6500* mpu, float* desiredAngularRates, int16_t* controlSignals) {
	controlSignals[0] = pidUpdateOutput(&rateModePitchController, mpu->angularVelocityX, desiredAngularRates[0]);
	controlSignals[1] = pidUpdateOutput(&rateModeRollController, mpu->angularVelocityY, desiredAngularRates[1]);
	controlSignals[2] = pidUpdateOutput(&rateModeYawController, mpu->angularVelocityZ, desiredAngularRates[2]);
}

/**
 * Calculates and outputs control signal based on the input and desired signals
 *
 * @param controller: pointer to a pidController struct
 * @param inputVal: the actual value of a signal
 * @param desiredVal: the desired value of a signal
 */
void angleController(MPU6500* mpu, float* desiredAngles, float* desiredAngularRates, int16_t* controlSignals) {
	desiredAngularRates[0] = -1.0f*pidUpdateOutput(&angleModePitchController, mpu->pitch, desiredAngles[0]);
	desiredAngularRates[1] = -1.0f*pidUpdateOutput(&angleModeRollController, mpu->roll, desiredAngles[1]);

	rateController(mpu, desiredAngularRates, controlSignals);
}

/**
 * Actuates the motors based on the controller output values for the pitch, roll, and yaw directions
 *
 * @param motorThrottle: pointer to a 4-element array containing the current speeds of the motors
 * @param rcThrottle: pointer to a 4-element array containing the throttle speed from the remote controller
 * @param controlSignals: pointer to a 3-element array containing controller compensation values for the pitch, roll, yaw directions
 */
void actuateMotors(uint8_t* currentMotorThrottle, uint8_t rcThrottle, int16_t* controlSignals) {
	currentMotorThrottle[0] = rcThrottle - controlSignals[0] + controlSignals[1] + controlSignals[2];
	currentMotorThrottle[1] = rcThrottle - controlSignals[0] - controlSignals[1] - controlSignals[2];
	currentMotorThrottle[2] = rcThrottle + controlSignals[0] + controlSignals[1] - controlSignals[2];
	currentMotorThrottle[3] = rcThrottle + controlSignals[0] - controlSignals[1] + controlSignals[2];

	// limit motor speeds to between 50 and 80
	for(uint8_t i = 0; i < 4; i++) {
		if(currentMotorThrottle[i] > 80) {
			currentMotorThrottle[i] = 80;
		}
		else if(currentMotorThrottle[i] < 50) {
			currentMotorThrottle[i] = 50;
		}
	}

	motorSetSpeed(frontRightMotor, currentMotorThrottle[0]);
	motorSetSpeed(frontLeftMotor, currentMotorThrottle[1]);
	motorSetSpeed(rearRightMotor, currentMotorThrottle[2]);
	motorSetSpeed(rearLeftMotor, currentMotorThrottle[3]);
}

/**
 * Maps raw input from joystick (0-4096) to a PWM output (50-100).
 *
 * TODO make this work for negative values?
 *
 * @param joystickVal: The raw value from the joystick, which should range from 0-4096.
 *
 * @returns: The mapped value to send over PWM, which will range from 50-100.
 */
uint8_t mapPWM(uint16_t joystickVal) {
  return (joystickVal >= 2200) ? (50*joystickVal) / 1896 - 8 : 50;
}
