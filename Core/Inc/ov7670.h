/*
 * ov7670.h
 *
 *  Created on: May 3, 2024
 *      Author: Cody Rupp
 */

#ifndef SRC_OV7670_H_
#define SRC_OV7670_H_
#include <stdint.h>

uint8_t ov7670WriteReg(uint8_t reg, uint8_t data);
uint8_t ov7670ReadReg(uint8_t reg, uint8_t* buf);

#define OV7670_I2C_ADDRESS (0x21 << 1)
#define I2C_TIMEOUT 100

#define OV7670_HREF_PORT GPIOB
#define OV7670_HREF_PIN  GPIO_PIN_4

#define OV7670_PCLK_PORT GPIOD
#define OV7670_PCLK_PIN  GPIO_PIN_2

// NOTE: THIS REGISTER LIST IS INCOMPLETE. Registers should be added as they are needed.
#define OV7670_BLUE 0x01
#define OV7670_RED  0x02

#define OV7670_COM7 0x12

#define OV7670_PID  0x0A
#define OV7670_VER  0x0B

#endif /* SRC_OV7670_H_ */
