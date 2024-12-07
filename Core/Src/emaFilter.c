/*
 * emaFilter.c
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#include "emaFilter.h"

void emaFilterInit(emaFilter* filter, float alpha) {
	// copy coefficient to struct
	filter->alpha = alpha;

	// initialize output to 0
	filter->outputData = 0.0f;
}

float emaFilterUpdate(emaFilter* filter, float inputData) {
	filter->outputData = (filter->alpha * inputData) - ((1 - filter->alpha) * filter->outputData);

	return filter->outputData;
}
