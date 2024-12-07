/*
 * pidController.h
 *
 *  Created on: May 10, 2024
 *      Author: jerem
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_

typedef struct {

	float kp;
	float ki;
	float kd;

	float desiredOutput;

	float currentError;
	float totalError;
	float controlOutput;

	uint16_t lastUpdated;

} pidController;

void pidControllerUpdate(float input);

#endif /* INC_PIDCONTROLLER_H_ */
