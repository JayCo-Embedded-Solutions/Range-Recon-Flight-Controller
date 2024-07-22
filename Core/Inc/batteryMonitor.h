/*
 * batteryMonitor.h
 *
 *  Created on: Jul 1, 2024
 *      Author: jerem
 */

#ifndef INC_BATTERYMONITOR_H_
#define INC_BATTERYMONITOR_H_

#include "stm32f4xx_hal.h"

// constants
#define ADC_MAX		4096
#define	ADC_MIN		0
#define BATT_MAX	12.6
#define BATT_MIN	0

// battery monitor ports and pins
#define BATT_CHECK_PORT		GPIOA
#define	BATT_CHECK_PIN		GPIO_PIN_3
#define BATT_DTCT_PORT		GPIOA
#define BATT_DTCT_PIN		GPIO_PIN_2

extern ADC_HandleTypeDef hadc1;

// battery monitor struct
typedef struct {

	// raw value from ADC
	uint16_t rawVal;

	// battery voltage
	float battVoltage;

} batteryMonitor;

void batteryMonitorInit(batteryMonitor* monitor);
void batteryMonitorEnable(batteryMonitor* monitor);
void batteryMonitorDisable(batteryMonitor* monitor);
float batteryMonitorUpdate(batteryMonitor* monitor);

#endif /* INC_BATTERYMONITOR_H_ */
