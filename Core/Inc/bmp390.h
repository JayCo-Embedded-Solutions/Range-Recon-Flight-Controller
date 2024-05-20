/*
 * bmp390.h
 *
 *  Created on: May 16, 2024
 *      Author: Cody Rupp
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

// Change I2C handle here
extern I2C_HandleTypeDef hi2c1;
#define BMP390_I2C_HANDLE &hi2c1

#define BMP390_I2C_ADDRESS 0x77 << 1

#define BMP390_CHIP_ID  0x00
#define BMP390_STATUS   0x03
#define BMP390_DATA_0   0x04
#define BMP390_DATA_1   0x05
#define BMP390_DATA_2   0x06
#define BMP390_DATA_3   0x07
#define BMP390_DATA_4   0x08
#define BMP390_DATA_5   0x09
#define BMP390_PWR_CTRL 0x1B
#define BMP390_OSR      0x1C
#define BMP390_ODR      0x1D
#define BMP390_CONFIG   0x1F
#define BMP390_PAR_T1_L 0x31
#define BMP390_PAR_T1_H 0x32
#define BMP390_PAR_T2_L 0x33
#define BMP390_PAR_T2_H 0x34
#define BMP390_PAR_T3   0x35
#define BMP390_PAR_P1_L 0x36
#define BMP390_PAR_P1_H 0x37
#define BMP390_PAR_P2_L 0x38
#define BMP390_PAR_P2_H 0x39
#define BMP390_PAR_P3   0x3A
#define BMP390_PAR_P4   0x3B
#define BMP390_PAR_P5_L 0x3C
#define BMP390_PAR_P5_H 0x3D
#define BMP390_PAR_P6_L 0x3E
#define BMP390_PAR_P6_H 0x3F
#define BMP390_PAR_P7   0x40
#define BMP390_PAR_P8   0x41
#define BMP390_PAR_P9_L 0x42
#define BMP390_PAR_P9_H 0x43
#define BMP390_PAR_P10  0x44
#define BMP390_PAR_P11  0x45

#define BMP390_DRDY_TEMP_MASK   0b01000000
#define BMP390_DRDY_PRESS_MASK  0b00100000

#define SEA_LEVEL_PRESSURE 101325

typedef struct bmp390 {
  // Temperature compensation coefficients
  uint16_t  t1;
  uint16_t  t2;
  int8_t    t3;

  // Pressure compensation coefficients
  int16_t   p1;
  int16_t   p2;
  int8_t    p3;
  int8_t    p4;
  uint16_t  p5;
  uint16_t  p6;
  int8_t    p7;
  int8_t    p8;
  int16_t   p9;
  int8_t    p10;
  int8_t    p11;

  // Last read temperature value (in degrees Celsius)
  float temp;

  // Last read pressure value (in Pascals)
  float pressure;

  // Last calculated altitude value (in meters)
  float alt;
} BMP390;

uint8_t bmp390Init(BMP390* bmp);
uint8_t bmp390Update(BMP390* bmp);
uint8_t bmp390UpdateTemperature(BMP390* bmp);
uint8_t bmp390UpdatePressureAltitude(BMP390* bmp);
float bmp390ConvertRawTemperature(uint32_t rawTemp, BMP390* bmp);
float bmp390ConvertRawPressure(uint32_t rawPressure, BMP390* bmp);
uint8_t bmp390WriteReg(uint8_t reg, uint8_t val);
uint8_t bmp390ReadReg(uint8_t reg, uint8_t* val);
uint8_t bmp390ReadRegMulti(uint8_t reg, uint8_t* vals, uint8_t size);

#endif /* INC_BMP390_H_ */
