/*
 * flightController.c
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#include "flightController.h"
#include "stm32f3xx_hal.h"
#include "math.h"

extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart4;

firFilter accelXDataLPF;
firFilter accelYDataLPF;
firFilter accelZDataLPF;

uint16_t gyroLastUpdate;
uint16_t pidLastUpdate;
uint16_t integral[3];

void initializeAccelFilters() {
	gyroLastUpdate = __HAL_TIM_GET_COUNTER(&htim16);
	pidLastUpdate = __HAL_TIM_GET_COUNTER(&htim16);
	integral[0] = 0;
	integral[1] = 0;
	integral[2] = 0;
	firFilterInit(&accelXDataLPF);
	firFilterInit(&accelYDataLPF);
	firFilterInit(&accelZDataLPF);
}

void imuExtractAndFilter(float* accelerometerData, float* gyroscopeData) {
	float accelDataRaw[3], gyroDataRaw[3];

	getAccelData(accelDataRaw);
	getGyroData(gyroDataRaw);

	accelerometerData[0] = firFilterUpdate(&accelXDataLPF, accelDataRaw[0]) - accelXOffset;
	accelerometerData[1] = firFilterUpdate(&accelYDataLPF, accelDataRaw[1]) - accelYOffset;
	accelerometerData[2] = firFilterUpdate(&accelZDataLPF, accelDataRaw[2]) - accelZOffset;

	gyroscopeData[0] = gyroDataRaw[0] - gyroXOffset;
	gyroscopeData[1] = gyroDataRaw[1] - gyroYOffset;
	gyroscopeData[2] = gyroDataRaw[2] - gyroZOffset;
}

void updateCraftAngles(float* accelerometerData, float* gyroscopeData, float* aircraftAngles) {

	imuExtractAndFilter(accelerometerData, gyroscopeData);

	float gyroAngles[3];
	float accelAngles[2];

	uint16_t dt = (__HAL_TIM_GET_COUNTER(&htim16) - gyroLastUpdate) / USecs2Secs;

	gyroAngles[0] = aircraftAngles[0] + (gyroscopeData[0] * dt) / GYRO_Prescale;
	gyroAngles[1] = aircraftAngles[1] + (gyroscopeData[1] * dt) / GYRO_Prescale;
	gyroAngles[2] = aircraftAngles[2] + (gyroscopeData[2] * dt) / GYRO_Prescale;

	accelAngles[0] = atan2(accelerometerData[1], accelerometerData[2]) * RAD2DEG;
	accelAngles[1] = atan2(-1 * accelerometerData[0], sqrt(accelerometerData[1]*accelerometerData[1] + accelerometerData[2]*accelerometerData[2])) * RAD2DEG;

	aircraftAngles[0] = 0.97f * gyroAngles[0] + 0.03f * accelAngles[0];		// complementary filter
	aircraftAngles[1] = 0.97f * gyroAngles[1] + 0.03f * accelAngles[1];
	aircraftAngles[2] = gyroAngles[2];

	gyroLastUpdate = __HAL_TIM_GET_COUNTER(&htim16);
}

void rateController(float* aircraftAngularRates, float* desiredAngularRates, int16_t* controlSignals) {
	float pitchError, rollError, yawError;

	char buf[1000];

	pitchError = aircraftAngularRates[0] - desiredAngularRates[0];
	rollError = aircraftAngularRates[1] - desiredAngularRates[1];
	yawError = aircraftAngularRates[2] - desiredAngularRates[2];

	integral[0] += pitchError;
	integral[1] += rollError;
	integral[2] += yawError;

	uint16_t dt = __HAL_TIM_GET_COUNTER(&htim16) - pidLastUpdate;

	int16_t pitchAdjust, rollAdjust, yawAdjust;

	pitchAdjust = (RATE_KP * pitchError) + (RATE_KI * integral[0]) / dt - (RATE_KD * pitchError * dt);
	rollAdjust = (RATE_KP * rollError) + (RATE_KI * integral[1]) / dt - (RATE_KD * rollError * dt);
	yawAdjust = (RATE_KP * yawError) + (RATE_KI * integral[2]) / dt - (RATE_KD * yawError * dt);

	controlSignals[0] = pitchAdjust;
	controlSignals[1] = rollAdjust;
	controlSignals[2] = yawAdjust;

//	sprintf(buf, "%0.1f, %0.1f, %0.1f \r\n",  pitchError, rollError, yawError);

	pidLastUpdate = __HAL_TIM_GET_COUNTER(&htim16);
}

void angleController(float* aircraftAngles, float* desiredAngles, int16_t* controlSignals) {
	float pitchError, rollError, yawError;

	char buf[1000];

	pitchError = aircraftAngles[0] - desiredAngles[0];
	rollError = aircraftAngles[1] - desiredAngles[1];
	yawError = aircraftAngles[2] - desiredAngles[2];

	uint16_t dt = __HAL_TIM_GET_COUNTER(&htim16) - pidLastUpdate;

	int16_t pitchAdjust, rollAdjust, yawAdjust;

	pitchAdjust = (RATE_KP * pitchError) + (RATE_KI * integral[0]) / dt - (RATE_KD * pitchError * dt);
	rollAdjust = (RATE_KP * rollError) + (RATE_KI * integral[1]) / dt - (RATE_KD * rollError * dt);
	yawAdjust = (RATE_KP * yawError) + (RATE_KI * integral[2]) / dt - (RATE_KD * yawError * dt);

	controlSignals[0] = pitchAdjust;
	controlSignals[1] = rollAdjust;
	controlSignals[2] = yawAdjust;

//	sprintf(buf, "%0.1f, %0.1f, %0.1f \r\n",  pitchError, rollError, yawError);
}

void actuateMotors(uint8_t* motorThrottle,uint8_t* rcThrottle, int16_t* controlSignals) {
	uint8_t adjustedSpeeds[4];

	char buf[1000];

	// assign motor speeds based on controller values
//	adjustedSpeeds[0] = rcThrottle[0] - controlSignals[0] + controlSignals[1] - controlSignals[2];
//	adjustedSpeeds[1] = rcThrottle[1] - controlSignals[0] - controlSignals[1] + controlSignals[2];
//	adjustedSpeeds[2] = rcThrottle[2] + controlSignals[0] + controlSignals[1] + controlSignals[2];
//	adjustedSpeeds[3] = rcThrottle[3] + controlSignals[0] - controlSignals[1] - controlSignals[2];
	adjustedSpeeds[0] = rcThrottle[0] - controlSignals[0] + controlSignals[1] + controlSignals[2];
	adjustedSpeeds[1] = rcThrottle[1] - controlSignals[0] - controlSignals[1] - controlSignals[2];
	adjustedSpeeds[2] = rcThrottle[2] + controlSignals[0] + controlSignals[1] - controlSignals[2];
	adjustedSpeeds[3] = rcThrottle[3] + controlSignals[0] - controlSignals[1] + controlSignals[2];

	// limit motor speeds to between 50 and 70
	for(uint8_t i = 0; i < 4; i++) {
		if(adjustedSpeeds[i] > 80) {
			adjustedSpeeds[i] = 80;
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
