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
}

/**
 * @brief: extracts and filters gyroscope and accelerometer data from the IMU
 *
 * @param accelerometerData: pointer to 3-element array containing accelerometer data for x, y, z axes
 * @param gyroscopeData: pointer to 3-element array containing gyroscope data for x, y, z axes
 *
 * @returns: none
 */
void imuExtractAndFilter(float* accelerometerData, float* gyroscopeData) {
	float accelDataRaw[3], gyroDataRaw[3];

	// extract raw gyroscope and accelerometer values
	getAccelData(accelDataRaw);
	getGyroData(gyroDataRaw);

	// filter accelerometer values and adjust for initial offset
	accelerometerData[0] = firFilterUpdate(&accelXDataLPF, accelDataRaw[0]) - accelXOffset;
	accelerometerData[1] = firFilterUpdate(&accelYDataLPF, accelDataRaw[1]) - accelYOffset;
	accelerometerData[2] = firFilterUpdate(&accelZDataLPF, accelDataRaw[2]) - accelZOffset;

	// adjust gyroscope values for initial offset
	gyroscopeData[0] = gyroDataRaw[0] - gyroXOffset;
	gyroscopeData[1] = gyroDataRaw[1] - gyroYOffset;
	gyroscopeData[2] = gyroDataRaw[2] - gyroZOffset;
}

/**
 * @brief: calculates and updates the absolute angles of the aircraft
 *
 * @param accelerometerData: pointer to 3-element array containing accelerometer data for x, y, z axes
 * @param gyroscopeData: pointer to 3-element array containing gyroscope data for x, y, z axes
 * @param aircraftAngles: pointer to a 3-element array containing absolute angles of the craft for the x, y, and z axes
 *
 * @returns: none
 */
void updateCraftAngles(float* accelerometerData, float* gyroscopeData, float* aircraftAngles) {

	// extract filtered IMU data
	imuExtractAndFilter(accelerometerData, gyroscopeData);

	float gyroAngles[3];
	float accelAngles[2];

	// determine change in time since last sample
	uint16_t dt = (__HAL_TIM_GET_COUNTER(&htim14) - gyroLastUpdate);

	// determine gyroscope angles via gyroscope data integration
	gyroAngles[0] = aircraftAngles[0] + (gyroscopeData[0] * dt) / USecs2Secs;
	gyroAngles[1] = aircraftAngles[1] + (gyroscopeData[1] * dt) / USecs2Secs;
	gyroAngles[2] = aircraftAngles[2] + (gyroscopeData[2] * dt) / USecs2Secs;

	// determine accelerometer angles using Euler equations
	accelAngles[0] = atan2(accelerometerData[1], accelerometerData[2]) * RAD2DEG;
	accelAngles[1] = atan2(-1 * accelerometerData[0], sqrt(accelerometerData[1]*accelerometerData[1] + accelerometerData[2]*accelerometerData[2])) * RAD2DEG;

	// fuse gyroscope and accelerometer data using a complementary filter
	aircraftAngles[0] = 0.98f * gyroAngles[0] + 0.02f * accelAngles[0];
	aircraftAngles[1] = 0.98f * gyroAngles[1] + 0.02f * accelAngles[1];
	aircraftAngles[2] = gyroAngles[2];

	// update gyro sample time
	gyroLastUpdate = __HAL_TIM_GET_COUNTER(&htim14);
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
//void angleController(float* aircraftAngles, float* desiredAngles, int16_t* controlSignals) {
//	float pitchError, rollError, yawError;
//
//	char buf[1000];
//
//	pitchError = aircraftAngles[0] - desiredAngles[0];
//	rollError = aircraftAngles[1] - desiredAngles[1];
//	yawError = aircraftAngles[2] - desiredAngles[2];
//
//	uint16_t dt = __HAL_TIM_GET_COUNTER(&htim14) - pidLastUpdate;
//
//	int16_t pitchAdjust, rollAdjust, yawAdjust;
//
//	pitchAdjust = (RATE_KP * pitchError) + (RATE_KI * integral[0]) / dt - (RATE_KD * pitchError * dt);
//	rollAdjust = (RATE_KP * rollError) + (RATE_KI * integral[1]) / dt - (RATE_KD * rollError * dt);
//	yawAdjust = (RATE_KP * yawError) + (RATE_KI * integral[2]) / dt - (RATE_KD * yawError * dt);
//
//	controlSignals[0] = pitchAdjust;
//	controlSignals[1] = rollAdjust;
//	controlSignals[2] = yawAdjust;
//
////	sprintf(buf, "%hd, %hd, %hd \r\n",  controlSignals[0], controlSignals[1], controlSignals[2]);
////	HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
////	HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
////	pidLastUpdate = __HAL_TIM_GET_COUNTER(&htim13);
//}

/**
 * @brief: actuates the motors based on the controller output values for the pitch, roll, and yaw directions
 *
 * @param motorThrottle: pointer to a 4-element array containing the current speeds of the motors
 * @param rcThrottle: pointer to a 4-element array containing the throttle speed from the remote controller
 * @param controlSignals: pointer to a 3-element array containing controller compensation values for the pitch, roll, yaw directions
 *
 * @returns: none
 */
void actuateMotors(uint8_t* motorThrottle, uint8_t* rcThrottle, int16_t* controlSignals) {
	uint8_t adjustedSpeeds[4];

	char buf[1000];

	adjustedSpeeds[0] = rcThrottle[0] - controlSignals[0] + controlSignals[1] + controlSignals[2];
	adjustedSpeeds[1] = rcThrottle[1] - controlSignals[0] - controlSignals[1] - controlSignals[2];
	adjustedSpeeds[2] = rcThrottle[2] + controlSignals[0] + controlSignals[1] - controlSignals[2];
	adjustedSpeeds[3] = rcThrottle[3] + controlSignals[0] - controlSignals[1] + controlSignals[2];

	// limit motor speeds to between 50 and 70
	for(uint8_t i = 0; i < 4; i++) {
		if(adjustedSpeeds[i] > 90) {
			adjustedSpeeds[i] = 90;
		}
		else if(adjustedSpeeds[i] < 50) {
			adjustedSpeeds[i] = 50;
		}
	}

	sprintf(buf, "%hd, %hd, %hd, %hd \r\n",  adjustedSpeeds[0], adjustedSpeeds[1], adjustedSpeeds[2], adjustedSpeeds[3]);
	HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	motorSetSpeed(frontRightMotor, adjustedSpeeds[0]);
	motorSetSpeed(frontLeftMotor, adjustedSpeeds[1]);
	motorSetSpeed(rearRightMotor, adjustedSpeeds[2]);
	motorSetSpeed(rearLeftMotor, adjustedSpeeds[3]);
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
