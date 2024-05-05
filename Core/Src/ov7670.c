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
 * Initializes register values of the OV7670 to enable RGB streaming.
 *
 * @returns: HAL status; will return early if any I2C transmission fails.
 */
HAL_StatusTypeDef ov7670Init() {
  return ov7670WriteReg(OV7670_COM7, 0b00000100); // Set output to raw RGB.
}

/**
 * Writes 1 byte of data to a particular register on the OV7670.
 *
 * NOTE: Can't use the HAL_I2C_Mem_Write() function because the camera doesn't support
 *  I2C "re-start" signals.
 *
 * @param reg: The register address to write to.
 * @param data: The data to write to the register.
 *
 * @returns: HAL status; will return early if any I2C transmission fails.
 */
HAL_StatusTypeDef ov7670WriteReg(uint8_t reg, uint8_t data) {
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(OV7670_I2C_HANDLE, OV7670_I2C_ADDRESS, &reg, 1, I2C_TIMEOUT);
  if (status != HAL_OK) {
    return status;
  }

  return HAL_I2C_Master_Transmit(OV7670_I2C_HANDLE, OV7670_I2C_ADDRESS, &data, 1, I2C_TIMEOUT);
}

/**
 * Reads 1 byte of data from a particular register on the OV7670.
 *
 * NOTE: Can't use the HAL_I2C_Mem_Read() function because the camera doesn't support
 *  I2C "re-start" signals.
 *
 * @param reg: The register address to read from.
 * @param buf: A pointer to where the register value should be stored.
 *
 * @returns: HAL status; will return early if any I2C transmission fails.
 */
HAL_StatusTypeDef ov7670ReadReg(uint8_t reg, uint8_t* buf) {
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(OV7670_I2C_HANDLE, OV7670_I2C_ADDRESS, &reg, 1, I2C_TIMEOUT);
  if (status != HAL_OK) {
    return status;
  }

  return HAL_I2C_Master_Receive(OV7670_I2C_HANDLE, OV7670_I2C_ADDRESS, buf, 1, I2C_TIMEOUT);
}
