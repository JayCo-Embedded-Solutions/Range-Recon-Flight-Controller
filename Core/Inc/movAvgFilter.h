/*
 * movAvgFilter.h
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#ifndef INC_MOVAVGFILTER_H_
#define INC_MOVAVGFILTER_H_

#include "stdint.h"

#define MOVING_AVG_FILTER_SIZE	2

typedef struct {
	// array to store most recent values
	float buffer[MOVING_AVG_FILTER_SIZE];

	// pointer to buffer
	uint8_t bufPtr;

	// output data
	float outputData;

} movAvgFilter;

void movAvgInit(movAvgFilter* filter);
float movAvgUpdate(movAvgFilter* filter, float inputData);

#endif /* INC_MOVAVGFILTER_H_ */
