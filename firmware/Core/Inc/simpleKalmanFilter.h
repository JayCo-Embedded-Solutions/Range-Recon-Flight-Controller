/*
 * simpleKalmanFilter.h
 *
 * Adapted from https://github.com/denyssene/SimpleKalmanFilter/tree/master.
 *
 *  Created on: May 30, 2024
 *      Author: Cody Rupp
 */

#ifndef SIMPLEKALMANFILTER_H_
#define SIMPLEKALMANFILTER_H_

typedef struct {
  float measuredError;
  float estimatedError;

  float q;

  float curEstimate;
  float lastEstimate;

  float kalmanGain;
} simpleKalmanFilter;

void kalmanInit(simpleKalmanFilter* filter, float mea_e, float est_e, float qVal);
void kalmanUpdateEstimate(simpleKalmanFilter* filter, float mea);

#endif /* SIMPLEKALMANFILTER_H_ */
