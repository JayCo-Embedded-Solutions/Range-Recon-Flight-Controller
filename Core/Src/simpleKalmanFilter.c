/*
 * simpleKalmanFilter.c
 *
 * Adapted from https://github.com/denyssene/SimpleKalmanFilter/tree/master.
 *
 *  Created on: May 30, 2024
 *      Author: Cody Rupp
 */
#include "simpleKalmanFilter.h"
#include "math.h"

/**
 * Initializes a simpleKalmanFilter struct with given values.
 *
 * @param filter: The address of the simpleKalmanFilter struct to be initialized.
 * @param mea_e: Measurement uncertainty.
 * @param est_e: Estimation uncertainty.
 * @param qVal: Process noise.
 */
void kalmanInit(simpleKalmanFilter* filter, float mea_e, float est_e, float qVal) {
  filter->measuredError = mea_e;
  filter->estimatedError = est_e;
  filter->q = qVal;
  filter->curEstimate = 0;
  filter->kalmanGain = 0;
  filter->lastEstimate = 0;
}

/**
 * Updates the current value estimate based on a measured value.
 *
 * @param filter: The address of the simpleKalmanFilter struct used.
 * @param mea: The measured value, used to update the filter.
 */
void kalmanUpdateEstimate(simpleKalmanFilter* filter, float mea) {
  filter->kalmanGain = filter->estimatedError / (filter->estimatedError + filter->measuredError);
  filter->curEstimate = filter->lastEstimate + filter->kalmanGain * (mea - filter->lastEstimate);
  filter->estimatedError = (1.0f - filter->kalmanGain) * filter->estimatedError + fabsf(filter->lastEstimate - filter->curEstimate) * filter->q;
  filter->lastEstimate = filter->curEstimate;
}
