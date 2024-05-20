/*
 * movingAvgFilter.h
 *
 *  Created on: May 19, 2024
 *      Author: jerem
 */

#ifndef INC_MOVINGAVGFILTER_H_
#define INC_MOVINGAVGFILTER_H_

#include "stdint.h"

#define bufSize 	4

typedef struct {

	uint8_t buffer[bufSize];
	uint8_t bufPtr;

	uint8_t filterOutput;

} movingAvgFilter;

void movingAvgFilterInit(movingAvgFilter* filter);
uint16_t movingAvgFilterUpdate(movingAvgFilter* filter, uint16_t inputData);

#endif /* INC_MOVINGAVGFILTER_H_ */
