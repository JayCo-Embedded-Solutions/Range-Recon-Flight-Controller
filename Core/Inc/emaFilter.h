/*
 * emaFilter.h
 *
 *  Created on: May 6, 2024
 *      Author: jerem
 */

#ifndef INC_EMAFILTER_H_
#define INC_EMAFILTER_H_

typedef struct {
	// filter coefficient
	float alpha;

	// filter output data
	float outputData;

} emaFilter;

void emaFilterInit(emaFilter* filter, float alpha);
float emaFilterUpdate(emaFilter* filter, float inputData);

#endif /* INC_EMAFILTER_H_ */
