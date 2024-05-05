/*
 * ov7670.c
 *
 *  Created on: May 3, 2024
 *      Author: Cody Rupp
 */

#include "ov7670.h"
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c1;
#define OV7670_I2C_HANDLE &hi2c1

/**
 * Writes 1 byte of data to a particular register on the OV7670.
 *
 * @param reg: The register address to write to.
 * @param data: The data to write to the register.
 *
 * @returns: 0 if successful, 1+ if failed.
 */
uint8_t ov7670WriteReg(uint8_t reg, uint8_t data) {
  return HAL_I2C_Mem_Write(OV7670_I2C_HANDLE, OV7670_I2C_ADDRESS, reg, 1, &data, 1, I2C_TIMEOUT);
}

/**
 * Reads 1 byte of data from a particular register on the OV7670.
 *
 * @param reg: The register address to read from.
 * @param buf: A pointer to where the register value should be stored.
 *
 * @returns: 0 if succesful, 1+ if failed.
 */
uint8_t ov7670ReadReg(uint8_t reg, uint8_t* buf) {
  return HAL_I2C_Mem_Read(OV7670_I2C_HANDLE, OV7670_I2C_ADDRESS, reg, 1, buf, 1, I2C_TIMEOUT);
}
