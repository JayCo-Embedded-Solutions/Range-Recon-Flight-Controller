/*
 * motorControl.h
 *
 *  Created on: Apr 28, 2024
 *      Author: Jeremy
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#define PULSE_MIN				50
#define PULSE_MAX				100

extern enum {
	frontRightMotor,
	frontLeftMotor,
	rearRightMotor,
	rearLeftMotor
} motor;

void motorSetSpeed(uint8_t selectedMotor, uint8_t speed);
void initializeMotors(void);
void setAllMotors(uint8_t speed);
void testIncrementSpeed(void);
void testIndividualAddress(void);

#endif /* INC_MOTORCONTROL_H_ */
