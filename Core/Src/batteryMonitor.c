/*
 * batteryMonitor.c
 *
 *  Created on: Jul 1, 2024
 *      Author: jerem
 */

#include "batteryMonitor.h"

void batteryMonitorInit(batteryMonitor* monitor) {
	monitor->battVoltage = 0.0f;
	monitor->rawVal = 0;
}

void batteryMonitorEnable(batteryMonitor* monitor) {
	HAL_GPIO_WritePin(BATT_CHECK_PORT, BATT_CHECK_PIN, GPIO_PIN_SET);
}

void batteryMonitorDisable(batteryMonitor* monitor) {
	HAL_GPIO_WritePin(BATT_CHECK_PORT, BATT_CHECK_PIN, GPIO_PIN_RESET);
}

float batteryMonitorUpdate(batteryMonitor* monitor) {

	// enable battery monitor circuit
	batteryMonitorEnable(monitor);

	// read from ADC to get rawVal
	HAL_ADC_Start(&hadc1);
	HAL_Delay(1);
	monitor->rawVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// disable battery monitor circuit
	batteryMonitorDisable(monitor);

	// convert to battery voltage reading
	monitor->battVoltage = BATT_MIN + (BATT_MAX - BATT_MIN) * (monitor->rawVal - ADC_MIN) / (ADC_MAX - ADC_MIN);

	return monitor->battVoltage;
}
