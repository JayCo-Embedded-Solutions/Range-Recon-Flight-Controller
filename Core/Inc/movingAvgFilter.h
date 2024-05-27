/*
 * movingAvgFilter.h
 *
 *  Created on: May 19, 2024
 *      Author: jerem
 */

#ifndef INC_MOVINGAVGFILTER_H_
#define INC_MOVINGAVGFILTER_H_

#include "stdint.h"

typedef struct {
  // Buffer to store the N most recent samples
	float* samples;

	// Points to the index after the last element sample buffer (this will be where the next incoming element is stored)
	float* endPtr;

	// Indicates how many samples in the buffer are valid
	uint8_t validSamples;

	// Max number of samples in the buffer
	uint8_t numSamples;
} movingAvgFilter;

void movingAvgFilterInit(movingAvgFilter* filter, uint8_t numSamplesToAverage);
float movingAvgFilterUpdate(movingAvgFilter* filter, float newSample);

#endif /* INC_MOVINGAVGFILTER_H_ */
