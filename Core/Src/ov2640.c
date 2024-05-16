/*
 * ov2640.c
 *
 * NOTE: This library is written specifically for the OV2640 with the ArduCAM Mini 2MP Plus shield.
 *        It likely will not work with the OV2640 alone.
 *
 *  Created on: May 12, 2024
 *      Author: Cody Rupp
 */
#include "ov2640.h"
#include <stdlib.h>

extern UART_HandleTypeDef huart2;

/**
 * Initializes register values for the OV2640 to output 320x240 JPEG images.
 *
 * @returns: The number of errors that occurred when setting the register values.
 */
uint8_t ov2640Init() {
  uint8_t errors = 0;

  // I have no idea what's going on here, but ArduCAM seems to think the following sequence of register writes is necessary.
  errors += ov2640WriteReg(0xff, 0x01);
  errors += ov2640WriteReg(0x12, 0x80);

  errors += ov2640WriteRegMulti(OV2640_JPEG_INIT);
  errors += ov2640WriteRegMulti(OV2640_YUV422);
  errors += ov2640WriteRegMulti(OV2640_JPEG);

  errors += ov2640WriteReg(0xff, 0x01);
  errors += ov2640WriteReg(0x15, 0x00);

  errors += ov2640WriteRegMulti(OV2640_320x240_JPEG);

  return errors;
}

/**
 * Captures a single frame from the camera and reads it into a buffer.
 *
 * @param frameBuf: The address of where to store the frame data.
 * @param numBytes: The address of where to store the number of bytes buffered.
 *
 * @returns: The number of errors that occurred during the transmission.
 */
uint8_t ov2640GetFrame(uint8_t** frameBuf, uint32_t* numBytes) {
  uint8_t errors = 0;

  // Clear capture done flag
  errors += ov2640ArduCAMWriteReg(OV2640_ARDUCAM_FIFO, 0b00000001);

  // Start frame capture
  errors += ov2640ArduCAMWriteReg(OV2640_ARDUCAM_FIFO, 0b00000010);

  // Wait for FIFO to finish buffering an entire frame
  uint8_t status = 0;
  while ((status & 0b00001000) != 0b00001000) {
    errors += ov2640ArduCAMReadReg(OV2640_ARDUCAM_STATUS, &status);
  }

  // Get number of bytes to read from FIFO
  uint32_t frameSize;
  errors += ov2640GetFrameSize(&frameSize);

  // Allocate space for frame
  *frameBuf = (uint8_t*)malloc(frameSize*sizeof(uint8_t));

  // Read frame into buffer
  HAL_GPIO_WritePin(OV2640_CS_PORT, OV2640_CS_PIN, RESET);
  uint8_t val, prevVal;
  uint8_t data = OV2640_ARDUCAM_BURST;
  errors += HAL_SPI_TransmitReceive(OV2640_SPI_HANDLE, &data, &val, 1, HAL_MAX_DELAY);

  // Receive JPEG header bytes
  data = 0x00;
  errors += HAL_SPI_TransmitReceive(OV2640_SPI_HANDLE, &data, &prevVal, 1, HAL_MAX_DELAY);
  errors += HAL_SPI_TransmitReceive(OV2640_SPI_HANDLE, &data, &val, 1, HAL_MAX_DELAY);

  // Ensure header data is received
  if ((prevVal != 0xFF) || (val != 0xD8)) {
    return errors + 1;
  }

  // Write data to buffer
  (*frameBuf)[0] = prevVal;
  (*frameBuf)[1] = val;

  // Transmit frame + ending bytes
  for (int i = 2; i < frameSize; i++) {
    prevVal = val;

    // Place new byte into buffer
    errors += HAL_SPI_TransmitReceive(OV2640_SPI_HANDLE, &data, &val, 1, HAL_MAX_DELAY);
    (*frameBuf)[i] = val;

    if ((val == 0xD9) && (prevVal == 0xFF)) {
      *numBytes = i + 1; // Need to add one to account for final byte
      break;
    }
  }
  HAL_GPIO_WritePin(OV2640_CS_PORT, OV2640_CS_PIN, SET);

  return errors;
}

/**
 * Gets the number of bytes stored in the FIFO.
 * Note that the FIFO may contain more bytes than just the image stored,
 *  and so this number is always an upper bound of the number of bytes in a single frame.
 *
 * @param size: A pointer to where the size should be stored.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t ov2640GetFrameSize(uint32_t* size) {
  uint8_t errors = 0;
  uint8_t size1 = 0, size2 = 0, size3 = 0;

  // Get low, mid, high bytes of size
  errors += ov2640ArduCAMReadReg(OV2640_ARDUCAM_SIZE_1, &size1);
  errors += ov2640ArduCAMReadReg(OV2640_ARDUCAM_SIZE_2, &size2);
  errors += ov2640ArduCAMReadReg(OV2640_ARDUCAM_SIZE_3, &size3);

  // Construct full unsigned int and write
  *size = ((uint32_t)size3 << 16) | ((uint16_t)size2 << 8) | size1;

  return errors;
}

/**
 * Iterates over a given register initialization table and writes each value to the corresponding register.
 * A register initialization table MUST have at least one element and {0xFF, 0xFF} as its last element.
 *
 * @param regVals: A pointer to the register initialization table to use.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t ov2640WriteRegMulti(const sensor_reg_t* regVals) {
  uint8_t errors = 0;

  uint8_t regAddr = regVals[0].reg;
  uint8_t regVal = regVals[0].val;
  for (uint8_t i = 1; !(regAddr == 0xFF && regVal == 0xFF); i++) {
    errors += ov2640WriteReg(regAddr, regVal);

    regAddr = regVals[i].reg;
    regVal = regVals[i].val;
  }

  errors += ov2640WriteReg(0xff, 0xff);

  return errors;
}

/**
 * Writes a single byte of data to a particular register on the OV2640.
 *
 * @param reg: The register address to write to.
 * @param val: The value to write to the register.
 *
 * @returns: 1 if error occurred during transmission, 0 if no errors.
 */
uint8_t ov2640WriteReg(uint8_t reg, uint8_t val) {
  uint8_t status = HAL_OK;

  // This needs to be a mem write as opposed to 2 individual writes (like with the SPI interface), based on the datasheet
  status = HAL_I2C_Mem_Write(OV2640_I2C_HANDLE, OV2640_I2C_ADDRESS, reg, 1, &val, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  return HAL_OK;
}

/**
 * Reads a single byte of data from a particular register on the OV2640.
 *
 * @param reg: The register address to read from.
 * @param val: A pointer to the place where the register value should be stored.
 *
 * @returns: 1 if error occurred during transmission, 0 if no errors.
 */
uint8_t ov2640ReadReg(uint8_t reg, uint8_t* val) {
  uint8_t status = HAL_OK;

  // Tell OV2640 which register we want to read from
  status = HAL_I2C_Master_Transmit(OV2640_I2C_HANDLE, OV2640_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  // Read data into given pointer
  status = HAL_I2C_Master_Receive(OV2640_I2C_HANDLE, OV2640_I2C_ADDRESS, val, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  return HAL_OK;
}

/**
 * Writes a single byte of data to a particular register on the ArduCAM SPI interface.
 *
 * @param reg: The register address to write to.
 * @param val: The value to write to the register.
 *
 * @returns: 1 if error occurred during transmission; 0 if no errors.
 */
uint8_t ov2640ArduCAMWriteReg(uint8_t reg, uint8_t val) {
  // Assert CS pin to begin transaction
  HAL_GPIO_WritePin(OV2640_CS_PORT, OV2640_CS_PIN, RESET);

  uint8_t status = HAL_OK;

  // Tell chip you want to write to the register
  uint8_t maskedReg = OV2640_WRITE_BITMASK | reg;
  status = HAL_SPI_Transmit(OV2640_SPI_HANDLE, &maskedReg, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  // Transmit data to write
  status = HAL_SPI_Transmit(OV2640_SPI_HANDLE, &val, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  // De-assert CS pin to end transaction
  HAL_GPIO_WritePin(OV2640_CS_PORT, OV2640_CS_PIN, SET);

  return HAL_OK;
}

/**
 * Reads a single byte of data from a particular register on the ArduCAM SPI interface.
 *
 * @param reg: The register address to read from.
 * @param val: A pointer to the address of where the register data should be stored.
 *
 * @returns: 1 if error occurred during transmission, 0 if no errors.
 */
uint8_t ov2640ArduCAMReadReg(uint8_t reg, uint8_t* val) {
  // Assert CS pin to begin transaction
  HAL_GPIO_WritePin(OV2640_CS_PORT, OV2640_CS_PIN, RESET);

  HAL_StatusTypeDef status = HAL_OK;

  // Tell chip you want to read from the register (no bitmask needed since 0 in bit[7] signals read)
  status = HAL_SPI_Transmit(OV2640_SPI_HANDLE, &reg, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  uint8_t test = 0;

  // Receive data returned from register
  status = HAL_SPI_Receive(OV2640_SPI_HANDLE, &test, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return 1;
  }

  *val = test;

  // De-assert CS pin to end transaction
  HAL_GPIO_WritePin(OV2640_CS_PORT, OV2640_CS_PIN, SET);

  return HAL_OK;
}
