/*
 * movingAvgFilter.c
 *
 *  Created on: May 19, 2024
 *      Author: jerem
 */

#include "movingAvgFilter.h"

void movingAvgFilterInit(movingAvgFilter* filter) {
	// initialize filter buffer and pointer to 0
	for(int i = 0; i<bufSize; i++) {
		filter->buffer[i] = 0;
	}
	filter->bufPtr = 0;

	filter->filterOutput = 0;
}

uint16_t movingAvgFilterUpdate(movingAvgFilter* filter, uint16_t inputData) {
	// store current input data in the buffer
	filter->buffer[filter->bufPtr] = inputData;

	// wrap bufPtr around if it has reached the end of the buffer
	filter->bufPtr = filter->bufPtr == bufSize ? 0 : filter->bufPtr++;

	// average the buffer values and return
	filter->filterOutput = 0;
	for(int i = 0; i<bufSize; i++) {
		filter->filterOutput += filter->buffer[i];
	}

	return filter->filterOutput / bufSize;
}
