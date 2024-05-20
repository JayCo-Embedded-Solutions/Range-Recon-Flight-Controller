/*
 * pidController.h
 *
 *  Created on: May 15, 2024
 *      Author: jerem
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_

#include "stdint.h"

#define USecs2Secs		1000000.0

typedef struct {

  float KP;
  float KI;
  float KD;

  float proportional;
  float integral;
  float derivative;

  float currentError;
  float prevError;
  float output;

  float dt;
  uint16_t lastUpdated;

} pidController;

void pidControllerInit(pidController* controller, float Kp, float Ki, float Kd);
float pidUpdateOutput(pidController* controller, float inputVal, float desiredVal);

#endif /* INC_PIDCONTROLLER_H_ */
