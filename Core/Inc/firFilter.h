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

void firFilterInit(firFilter* filter);
float firFilterUpdate(firFilter*, float inputData);

#endif /* INC_FIRFILTER_H_ */
