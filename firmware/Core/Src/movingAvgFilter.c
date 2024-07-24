/*
 * movingAvgFilter.c
 *
 *  Created on: May 19, 2024
 *      Author: jerem
 */

#include "movingAvgFilter.h"

/**
 * TODO
 */
void movingAvgFilterInit(movingAvgFilter* filter, uint8_t numSamplesToAverage) {
  filter->numSamples = numSamplesToAverage;

  // Allocate buffer to store N most recent samples
	filter->samples = (float*)calloc(numSamplesToAverage, sizeof(float));

	filter->endPtr = filter->samples;

	filter->validSamples = 0;
}

/**
 * Inserts the new sample into the sample buffer and then returns the average value of all samples.
 * NOTE: If the buffer is full, this will overwrite the oldest sample.
 *
 * @param filter: The address of the filter to be updated.
 * @param newSample: The sample to add to the buffer.
 *
 * @returns: The updated average of all sample values.
 */
float movingAvgFilterUpdate(movingAvgFilter* filter, float newSample) {
  // Insert new sample into buffer
  *(filter->endPtr) = newSample;

  // If end pointer is at ending address of the allocated buffer, circle back to beginning
  (filter->endPtr == filter->samples + filter->numSamples - 1) ? filter->endPtr = filter->samples : filter->endPtr++;

  // Update number of valid samples if necessary
  if (filter->validSamples < filter->numSamples) filter->validSamples++;

  // Calculate & return new average, making sure to not count invalid samples
  float sum = 0.0f;
  for (uint8_t i = 0; i < filter->validSamples; i++) {
    sum += filter->samples[i]; // Note that the order in which we sum samples doesn't need to match the order when they were added to the buffer.
  }

  return sum / filter->validSamples;
}
