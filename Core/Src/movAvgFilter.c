/*
 * movAvgFilter.c
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#include "movAvgFilter.h"
#include "stdint.h"

void movAvgInit(movAvgFilter* filter) {
	// initialize all buffer elements to 0
	for(uint8_t i = 0; i < MOVING_AVG_FILTER_SIZE; i++) {
		filter->buffer[i] = 0.0f;
	}

	// initialize bufPtr to 0
	filter->bufPtr = 0;

	// initialize output to 0
	filter->outputData = 0.0f;
}

float movAvgUpdate(movAvgFilter* filter, float inputData) {
	// process inputData
	filter->buffer[filter->bufPtr] = inputData;

	// increment/wrap buffer pointer
	filter->bufPtr++;
	if(filter->bufPtr == MOVING_AVG_FILTER_SIZE) {
		filter->bufPtr = 0;
	}

	filter->outputData = 0.0f;

	for(uint8_t i = 0; i < MOVING_AVG_FILTER_SIZE; i++) {
		filter->outputData += filter->buffer[i];
	}

	return filter->outputData / MOVING_AVG_FILTER_SIZE;
}
