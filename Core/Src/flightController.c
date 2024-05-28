/*
 * flightController.c
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#include "flightController.h"

extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart4;

firFilter accelXDataLPF;
firFilter accelYDataLPF;
firFilter accelZDataLPF;

pidController rateModePitchController;
pidController rateModeRollController;
pidController rateModeYawController;

pidController angleModePitchController;
pidController angleModeRollController;

uint16_t gyroLastUpdate;

/**
 * @brief: initialize all necessary elements of the flight controller (gyro update time, accelerometer filters, PID controllers)
 *
 * @returns: none
 */
void flightControllerInit() {
	// get current time for gyroscope update
	gyroLastUpdate = __HAL_TIM_GET_COUNTER(&htim14);

	// initialize FIR low-pass filters for accelerometer data
	firFilterInit(&accelXDataLPF);
	firFilterInit(&accelYDataLPF);
	firFilterInit(&accelZDataLPF);

	// initialize PID controllers for the rate mode controller
	pidControllerInit(&rateModePitchController, RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
	pidControllerInit(&rateModeRollController, RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
	pidControllerInit(&rateModeYawController, RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD);

	// initialize PID controllers for the angle mode controller
	pidControllerInit(&angleModePitchController, ANGLE_PITCH_KP, ANGLE_PITCH_KI, ANGLE_PITCH_KD);
	pidControllerInit(&angleModeRollController, ANGLE_ROLL_KP, ANGLE_ROLL_KI, ANGLE_ROLL_KD);
}

/**
 * @brief: extracts and filters gyroscope and accelerometer data from the IMU
 *
 * @param accelerometerData: pointer to 3-element array containing accelerometer data for x, y, z axes
 * @param gyroscopeData: pointer to 3-element array containing gyroscope data for x, y, z axes
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t imuExtractAndFilter(float* accelerometerData, float* gyroscopeData, MPU6500* mpu) {
  uint8_t errors = 0;

	// extract raw gyroscope and accelerometer values
	errors += updateAcceleration(mpu);
	errors += updateAngularVelocity(mpu);

	// filter accelerometer values and adjust for initial offset
	accelerometerData[0] = firFilterUpdate(&accelXDataLPF, mpu->accelerationX, FIR_IMPULSE_RESPONSE_10HZ) - mpu->accelOffsetX;
	accelerometerData[1] = firFilterUpdate(&accelYDataLPF, mpu->accelerationY, FIR_IMPULSE_RESPONSE_10HZ) - mpu->accelOffsetY;
	accelerometerData[2] = firFilterUpdate(&accelZDataLPF, mpu->accelerationZ, FIR_IMPULSE_RESPONSE_10HZ) - mpu->accelOffsetZ;

	// adjust gyroscope values for initial offset
	gyroscopeData[0] = mpu->angularVelocityX - mpu->gyroOffsetX;
	gyroscopeData[1] = mpu->angularVelocityY - mpu->gyroOffsetY;
	gyroscopeData[2] = mpu->angularVelocityZ - mpu->gyroOffsetZ;

	return errors;
}

/**
 * @brief: calculates and updates the absolute angles of the aircraft
 *
 * @param accelerometerData: pointer to 3-element array containing accelerometer data for x, y, z axes
 * @param gyroscopeData: pointer to 3-element array containing gyroscope data for x, y, z axes
 * @param aircraftAngles: pointer to a 3-element array containing absolute angles of the craft for the x, y, and z axes
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t updateCraftAngles(float* accelerometerData, float* gyroscopeData, float* aircraftAngles, MPU6500* mpu) {
  uint8_t errors = 0;

	// extract filtered IMU data
	errors += imuExtractAndFilter(accelerometerData, gyroscopeData, mpu);

	float gyroAngles[2];
	float accelAngles[2];

	// determine change in time since last sample
	uint16_t dt = (__HAL_TIM_GET_COUNTER(&htim14) - gyroLastUpdate);

	// determine gyroscope angles via gyroscope data integration
	gyroAngles[0] = aircraftAngles[0] + (gyroscopeData[0] * dt) / USecs2Secs;
	gyroAngles[1] = aircraftAngles[1] + (gyroscopeData[1] * dt) / USecs2Secs;

	// determine accelerometer angles using Euler equations
	accelAngles[0] = atan2(accelerometerData[1], accelerometerData[2]) * RAD2DEG;
	accelAngles[1] = atan2(-1 * accelerometerData[0], sqrt(accelerometerData[1]*accelerometerData[1] + accelerometerData[2]*accelerometerData[2])) * RAD2DEG;

	// fuse gyroscope and accelerometer data using a complementary filter
	aircraftAngles[0] = 0.98f * gyroAngles[0] + 0.02f * accelAngles[0];
	aircraftAngles[1] = 0.98f * gyroAngles[1] + 0.02f * accelAngles[1];

	// update gyro sample time
	gyroLastUpdate = __HAL_TIM_GET_COUNTER(&htim14);

	return errors;
}

/**
 * @brief: determines required adjustments in the x, y, and z directions based on actual and desired angular rates of the aircraft
 *
 * @param aircraftAngularRates: pointer to a 3-element array containing the current angular acceleration rates in the x, y, z directions
 * @param desiredAngularRates: pointer to a 3-element array containing the desired angular acceleration rates in the x, y, z directions
 * @param controlSignals: pointer to a 3-element array containing the required compensation in the x, y, z directions
 *
 * @returns: none
 */
void rateController(float* aircraftAngularRates, float* desiredAngularRates, int16_t* controlSignals) {

	controlSignals[0] = pidUpdateOutput(&rateModePitchController, aircraftAngularRates[0], desiredAngularRates[0]);
	controlSignals[1] = pidUpdateOutput(&rateModeRollController, aircraftAngularRates[1], desiredAngularRates[1]);
	controlSignals[2] = pidUpdateOutput(&rateModeYawController, aircraftAngularRates[2], desiredAngularRates[2]);

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
void angleController(float* aircraftAngles, float* desiredAngles, float* aircraftAngularRates, float* desiredAngularRates, int16_t* controlSignals) {

	desiredAngularRates[0] = -1.0f*pidUpdateOutput(&angleModePitchController, aircraftAngles[0], desiredAngles[0]);
	desiredAngularRates[1] = -1.0f*pidUpdateOutput(&angleModeRollController, aircraftAngles[1], desiredAngles[1]);

	rateController(aircraftAngularRates, desiredAngularRates, controlSignals);

}

/**
 * @brief: actuates the motors based on the controller output values for the pitch, roll, and yaw directions
 *
 * @param motorThrottle: pointer to a 4-element array containing the current speeds of the motors
 * @param rcThrottle: pointer to a 4-element array containing the throttle speed from the remote controller
 * @param controlSignals: pointer to a 3-element array containing controller compensation values for the pitch, roll, yaw directions
 *
 * @returns: none
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
 * @param joystickVal: The raw value from the joystick, which should range from 0-4096.
 *
 * @returns: The mapped value to send over PWM, which will range from 50-100.
 */
uint8_t mapPWM(uint16_t joystickVal) {
  return (joystickVal >= 2200) ? (50*joystickVal) / 1896 - 8 : 50;
}
