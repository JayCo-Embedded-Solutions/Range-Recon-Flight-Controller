/*
 * FirFilter.c
 *
 *  Created on: May 5, 2024
 *      Author: jerem
 */

#include "firFilter.h"

static float firImpulseResponse[FIR_FILTER_LENGTH] = { 0.01919f, 0.02748f, 0.04323f, 0.06096f, 0.07885f, 0.09485f,
													   0.10689f, 0.11336f, 0.11336f, 0.10689f, 0.09485f, 0.07885f,
													   0.06096f, 0.04323f, 0.02748f, 0.01919f};

void firFilterInit(firFilter* filter) {
	// initialize buffer to 0
	for(uint8_t i = 0; i < FIR_FILTER_LENGTH; i++) {
		filter->circBuf[i] = 0.0f;
	}

	// initialize buffer pointer to 0
	filter->bufIndex = 0;

	// initialize filter output to 0
	filter->outputData = 0.0f;
}

float firFilterUpdate(firFilter* filter, float inputData) {
	// store input sample in circular buffer
	filter->circBuf[filter->bufIndex] = inputData;

	// increment buffer pointer/wrap around
	if(filter->bufIndex == FIR_FILTER_LENGTH - 1) {
		filter->bufIndex = 0;
	} else {
		filter->bufIndex++;
	}

	// compute output data via convolution
	filter->outputData = 0.0f;

	uint8_t sumIndex = filter->bufIndex;

	for(uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		if(sumIndex == 0) {
			sumIndex = FIR_FILTER_LENGTH;
		} else {
			sumIndex--;
		}

		filter->outputData += firImpulseResponse[n] * filter->circBuf[sumIndex];
	}

	return filter->outputData;
}
