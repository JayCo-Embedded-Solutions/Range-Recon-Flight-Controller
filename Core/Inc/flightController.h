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

#define RATE_KP				0.23
#define RATE_KI				0
#define RATE_KD				0

#define RAD2DEG 		57.3
#define USecs2Secs		1000000
#define GYRO_Prescale 	55000.0

extern uint16_t integral[3];

extern float accelData[3];
extern float gyroData[3];

extern float craftAngles[3];

extern firFilter accelXDataLPF;
extern firFilter accelYDataLPF;
extern firFilter accelZDataLPF;

extern uint16_t gyroLastUpdate;

void initializeAccelFilters();
void imuExtractAndFilter(float* accelerometerData, float* gyroscopeData);
void updateCraftAngles(float* acceleromterData, float* gyroscopeData, float* aircraftAngles);
void rateController(float* aircraftAngularRates, float* desiredAngularRates, int16_t* controlSignals);
void angleController(float* aircraftAngles, float* desiredAngles, int16_t* controlSignals);
void actuateMotors(uint8_t* motorThrottle,uint8_t* rcThrottle, int16_t* controlSignals);

#endif /* INC_FLIGHTCONTROLLER_H_ */
