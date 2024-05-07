/*
 * ov7670.c
 *
 *  Created on: May 3, 2024
 *      Author: Cody Rupp
 */

#include "ov7670.h"

/**
 * Initializes register values of the OV7670 to enable RGB streaming.
 *
 * @returns: HAL status; will return early if any I2C transmission fails.
 */
HAL_StatusTypeDef ov7670Init() {
  HAL_StatusTypeDef status;

//  // Write default values
//  for (int i = 0; i < 0xff; i++) {
//    // 0xff is the "end signal" of the array, as seen in the header file.
//    if (ov7670_default_regs[i].reg_num == 0xff) {
//      break;
//    }
//
//    status = ov7670WriteReg(ov7670_default_regs[i].reg_num, ov7670_default_regs[i].value);
//    if (status != HAL_OK) {
//      return status;
//    }
//  }

  // Configure output to be RGB565 format
  for (int i = 0; i < 0xff; i++) {
    // 0xff is the "end signal" of the array, as seen in the header file.
    if (ov7670_fmt_rgb565[i].reg_num == 0xff) {
      break;
    }

    status = ov7670WriteReg(ov7670_fmt_rgb565[i].reg_num, ov7670_fmt_rgb565[i].value);
    if (status != HAL_OK) {
      return status;
    }
  }

  // Configure output to be QQVGA
  for (int i = 0; i < 0xff; i++) {
    // 0xff is the "end signal" of the array, as seen in the header file.
    if (ov7670_qqvga_regs[i].reg_num == 0xff) {
      break;
    }

    status = ov7670WriteReg(ov7670_qqvga_regs[i].reg_num, ov7670_qqvga_regs[i].value);
    if (status != HAL_OK) {
      return status;
    }
  }

  return HAL_OK;
}

/**
 * Reads all pixel values into a given buffer.
 * Note that we divide by 2 here to get the number of uint32_t's (since the actual output is uint16_t's).
 *
 * @returns: HAL status representing whether or not the DMA initialization attempt was successful.
 */
HAL_StatusTypeDef ov7670GetFrame(uint16_t* frameBuf) {
  HAL_StatusTypeDef status = HAL_DCMI_Start_DMA(OV7670_DCMI, DCMI_MODE_SNAPSHOT, (uint32_t)frameBuf, OV7670_NUM_ROWS*OV7670_NUM_COLS/2);

  status |= HAL_DCMI_Suspend(OV7670_DCMI);
  status |= HAL_DCMI_Stop(OV7670_DCMI);

  return status;
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
