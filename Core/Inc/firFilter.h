/*
 * FIR_FILTER.h
 *
 *  Created on: May 5, 2024
 *      Author: jerem
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

#include "stdint.h"

#define FIR_FILTER_LENGTH 	16

typedef struct {
	// circular buffer and pointer
	float circBuf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	// filter output data
	float outputData;
} firFilter;

//// filter coefficients for 16-tap Butterworth filter with a cutoff frequency of 10Hz
//static const float FIR_IMPULSE_RESPONSE_10HZ[FIR_FILTER_LENGTH] = { 0.01919f, 0.02748f, 0.04323f, 0.06096f, 0.07885f, 0.09485f,
//                             0.10689f, 0.11336f, 0.11336f, 0.10689f, 0.09485f, 0.07885f,
//                             0.06096f, 0.04323f, 0.02748f, 0.01919f};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 50 Hz

* 0 Hz - 2 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = 0.00002683482613004398 dB

* 17 Hz - 25 Hz
  gain = 0
  desired attenuation = -0.1 dB
  actual attenuation = -68.87913502517608 dB

*/

#define FILTER_TAP_NUM 16

static const float FIR_IMPULSE_RESPONSE_10HZ[FIR_FILTER_LENGTH] = {
  0.0015358138418116386,
  0.0019479632811869657,
  0.011621859286325368,
  0.003695627464742353,
  0.05010588973259068,
  0.022741961995029886,
  0.1713047117251835,
  0.0932055311098614,
  0.0932055311098614,
  0.1713047117251835,
  0.022741961995029886,
  0.05010588973259068,
  0.003695627464742353,
  0.011621859286325368,
  0.0019479632811869657,
  0.0015358138418116386
};




/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 100 Hz

* 0 Hz - 5 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = 4.308190802500044 dB

* 10 Hz - 50 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -25.00895996171625 dB

*/

#define FILTER_TAP_NUM 16

static const float FIR_IMPULSE_RESPONSE_8HZ[FIR_FILTER_LENGTH] = {
  -0.011933429474028192,
  0.03470823610169373,
  0.04683311482842448,
  0.06870524786692148,
  0.0930179268104798,
  0.11549273378434864,
  0.1326916997836216,
  0.14200304400079525,
  0.14200304400079525,
  0.1326916997836216,
  0.11549273378434864,
  0.0930179268104798,
  0.06870524786692148,
  0.04683311482842448,
  0.03470823610169373,
  -0.011933429474028192
};



void firFilterInit(firFilter* filter);
float firFilterUpdate(firFilter* filter, float inputData, const float* firImpulseResponse);

#endif /* INC_FIRFILTER_H_ */
