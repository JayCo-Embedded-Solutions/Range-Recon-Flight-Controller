/*
 * ov2640.h
 *
 *  Created on: May 12, 2024
 *      Author: Cody Rupp
 */

#ifndef INC_OV2640_H_
#define INC_OV2640_H_

#include "stm32f4xx_hal.h"
#include "ov2640_reg_init.h"

extern I2C_HandleTypeDef hi2c1;
#define OV2640_I2C_HANDLE &hi2c1
#define OV2640_I2C_ADDRESS 0x60

extern SPI_HandleTypeDef hspi1;
#define OV2640_SPI_HANDLE &hspi1

#define OV2640_CS_PORT  GPIOB
#define OV2640_CS_PIN   GPIO_PIN_6

#define OV2640_WRITE_BITMASK 0b10000000

#define OV2640_BUFFER_MAX_SIZE 4096

#define OV2640_ARDUCAM_CCR    0x01
#define OV2640_ARDUCAM_FIFO   0x04
#define OV2640_ARDUCAM_BURST  0x3C
#define OV2640_ARDUCAM_STATUS 0x41
#define OV2640_ARDUCAM_SIZE_1 0x42
#define OV2640_ARDUCAM_SIZE_2 0x43
#define OV2640_ARDUCAM_SIZE_3 0x44

uint8_t ov2640Init();
uint8_t ov2640GetFrame(uint8_t** frameBuf, uint32_t* numBytes);
uint8_t ov2640GetFrameSize(uint32_t* size);
uint8_t ov2640WriteRegMulti(const sensor_reg_t* regVals);
uint8_t ov2640WriteReg(uint8_t reg, uint8_t val);
uint8_t ov2640ReadReg(uint8_t reg, uint8_t* val);
uint8_t ov2640ArduCAMWriteReg(uint8_t reg, uint8_t val);
uint8_t ov2640ArduCAMReadReg(uint8_t reg, uint8_t* val);

#endif /* INC_OV2640_H_ */
