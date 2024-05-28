/*
 * bmp390.c
 *
 *  Created on: May 16, 2024
 *      Author: Cody Rupp
 *
 * This file contains a library for the BMP390 temperature/pressure sensor.
 * USAGE (assuming hardware is properly connected):
 *    1. Create a BMP390 struct.
 *    2. Call bmp390Init(), passing in the address of the struct you created.
 *          This initializes the sensor's sampling rate and filter coefficients and such.
 *    3. Call bmp390Update(), passing in the struct address.
 *          This will actually read the sensor values into the struct.
 *    4. Get the altitude/pressure/temperature through the BMP struct's corresponding members ("alt", "pressure", and "temp").
 *    5. Continue to call bmp390Update() every time you want to get fresh sensor readings.
 */
#include "bmp390.h"
#include <math.h>

/**
 * Initializes certain registers to non-default settings on the BMP390,
 *    and reads the necessary temp/pressure conversion values into the given struct.
 * Must be called before using any other functions in this library.
 * Since this will be used in a drone, the values here are mostly from section 3.5 of the BMP390 datasheet.
 *
 * @param bmp: A pointer to a BMP390 struct to be initialized.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t bmp390Init(BMP390* bmp) {
  uint8_t errors = 0;

  // Set pressure oversampling to x8
  errors += bmp390WriteReg(BMP390_OSR, 0b00000011);

  // Set output data rate to 50 Hz
  errors += bmp390WriteReg(BMP390_ODR, 0b00000010);

  // Set infinite impulse reponse (IIR) filter coefficient to 3
  errors += bmp390WriteReg(BMP390_CONFIG, 0b00000100);

  // Enable pressure sensor + temperature sensor in normal mode
  errors += bmp390WriteReg(BMP390_PWR_CTRL, 0b00110011);

  uint8_t lowByte, highByte;

  // Read temperature conversion coefficients into BMP390 struct
  errors += bmp390ReadReg(BMP390_PAR_T1_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_T1_H, &highByte);
  bmp->t1 = ((uint16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_T2_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_T2_H, &highByte);
  bmp->t2 = ((uint16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_T3, (uint8_t*)&(bmp->t3));

  // Read pressure conversion coefficients into BMP390 struct
  errors += bmp390ReadReg(BMP390_PAR_P1_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_P1_H, &highByte);
  bmp->p1 = ((int16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_P2_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_P2_H, &highByte);
  bmp->p2 = ((int16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_P3, (uint8_t*)&(bmp->p3));

  errors += bmp390ReadReg(BMP390_PAR_P4, (uint8_t*)&(bmp->p4));

  errors += bmp390ReadReg(BMP390_PAR_P5_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_P5_H, &highByte);
  bmp->p5 = ((uint16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_P6_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_P6_H, &highByte);
  bmp->p6 = ((uint16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_P7, (uint8_t*)&(bmp->p7));

  errors += bmp390ReadReg(BMP390_PAR_P8, (uint8_t*)&(bmp->p8));

  errors += bmp390ReadReg(BMP390_PAR_P9_L, &lowByte);
  errors += bmp390ReadReg(BMP390_PAR_P9_H, &highByte);
  bmp->p9 = ((int16_t)highByte << 8) | lowByte;

  errors += bmp390ReadReg(BMP390_PAR_P10, (uint8_t*)&(bmp->p10));

  errors += bmp390ReadReg(BMP390_PAR_P11, (uint8_t*)&(bmp->p11));

  return errors;
}

/**
 * Updates the temperature, pressure, and altitude values stored in a given BMP struct.
 *
 * NOTE: This function MUST be called before reading any values from the struct.
 *       Also note that the bmp390Init() function should be called prior to this.
 *
 * @param bmp: The address of the BMP sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t bmp390Update(BMP390* bmp) {
  uint8_t errors = 0;

  errors += bmp390UpdateTemperature(bmp);
  errors += bmp390UpdatePressureAltitude(bmp);

  return errors;
}

/**
 * Reads the BMP390's temperature sensor and updates the value in a BMP struct.
 * Units: Degrees Celsius.
 *
 * @param bmp: The address of the BMP sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t bmp390UpdateTemperature(BMP390* bmp) {
  uint8_t errors = 0;

  // Wait for data to be ready
  uint8_t status = 0;
  errors += bmp390ReadReg(BMP390_STATUS, &status);
  while (!(status & BMP390_DRDY_TEMP_MASK)) {
    errors += bmp390ReadReg(BMP390_STATUS, &status);
  }

  // Get temperature data
  uint8_t tempVals[3];
  errors += bmp390ReadRegMulti(BMP390_DATA_3, tempVals, 3);

  // Merge individual bytes into raw temperature
  uint32_t rawTemp = ((uint32_t)tempVals[2] << 16) | ((uint16_t)tempVals[1] << 8) | tempVals[0];

  // Update value in struct
  bmp->temp = bmp390ConvertRawTemperature(rawTemp, bmp);

  return errors;
}

/**
 * Reads the BMP390's pressure sensor and updates the value in a BMP struct.
 * Also calculates altitude based on the pressure value and stores it in the struct.
 * Pressure units: Pascals.
 * Altitude units: Meters.
 *
 * NOTE: bmp390UpdateTemperature() MUST be called prior to calling this function.
 *
 * @param bmp: The address of the BMP sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t bmp390UpdatePressureAltitude(BMP390* bmp) {
  uint8_t errors = 0;

  // Wait for data to be ready
  uint8_t status = 0;
  errors += bmp390ReadReg(BMP390_STATUS, &status);
  while (!(status & BMP390_DRDY_PRESS_MASK)) {
    errors += bmp390ReadReg(BMP390_STATUS, &status);
  }

  // Get pressure data
  uint8_t pressureVals[3];
  errors += bmp390ReadRegMulti(BMP390_DATA_0, pressureVals, 3);

  // Merge individual bytes into output pressure
  uint32_t rawPressure = ((uint32_t)pressureVals[2] << 16) | ((uint16_t)pressureVals[1] << 8) | pressureVals[0];

  // Update values in struct
  bmp->pressure = bmp390ConvertRawPressure(rawPressure, bmp);

  // Formula adapted from NOAA's Pressure Altitude equation: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
  bmp->alt = 44307.69396 * (1 - powf(bmp->pressure / 101325, 0.190284));

  return errors;
}

/**
 * Converts the raw temperature sensor output to degrees Celsius.
 * Uses the compensation values stored in the sensor,
 *    and the equations from sections 8.4-8.6 in the datasheet.
 *
 * @param rawTemp: The raw temperature sensor data.
 * @param bmp: The address of the BMP sensor struct.
 *
 * @returns: The converted raw sensor output, in degrees Celsius.
 */
float bmp390ConvertRawTemperature(uint32_t rawTemp, BMP390* bmp) {
  /*
   * The following code is admittedly pretty unreadable,
   *    but just trust that these equations work.
   *
   * If you're following along in the datasheet,
   *    I've commented the powers of 2 that I used to get the
   *    calibration coefficients in floating point format (see section 8.4).
   */
  float partial_data1 = (float)rawTemp - ((float)(bmp->t1) * 256); // 2^8
  float partial_data2 = partial_data1 * ((float)(bmp->t2) / 1073741824); // 2^30

  float convertedTemp = partial_data2 + (partial_data1 * partial_data1) * ((float)(bmp->t3) / 281474976710656); // 2^48

  return convertedTemp;
}

/**
 * Converts the raw pressure sensor output to Pascals.
 * Uses the compensation values stored in the sensor,
 *    and the equations from sections 8.4-8.6 in the datasheet.
 *
 * @param rawPressure: The raw pressure sensor data.
 * @param bmp: The address of the BMP sensor struct.
 *
 * @returns: The converted raw sensor output, in Pascals.
 */
float bmp390ConvertRawPressure(uint32_t rawPressure, BMP390* bmp) {
  /*
   * The following code is admittedly pretty unreadable,
   *    but just trust that these equations work.
   *
   * If you're following along in the datasheet,
   *    I've commented the powers of 2 that I used to get the
   *    calibration coefficients in floating point format (see section 8.4).
   */
  float partial_data1 = ((float)(bmp->p6) / 64) * bmp->temp; // 2^6
  float partial_data2 = ((float)(bmp->p7) / 256) * (bmp->temp * bmp->temp); // 2^8
  float partial_data3 = ((float)(bmp->p8) / 32768) * (bmp->temp * bmp->temp * bmp->temp); // 2^15
  float partial_out1 = ((float)(bmp->p5) * 8) + partial_data1 + partial_data2 + partial_data3; // 2^3

  partial_data1 = (((float)(bmp->p2) - 16384) / 536870912) * bmp->temp; // 2^14, 2^29
  partial_data2 = ((float)(bmp->p3) / 4294967296) * (bmp->temp * bmp->temp); // 2^32
  partial_data3 = ((float)(bmp->p4) / 137438953472) * (bmp->temp * bmp->temp * bmp->temp); // 2^37
  float partial_out2 = (float)rawPressure * ((((float)(bmp->p1) - 16384) / 1048576) + partial_data1 + partial_data2 + partial_data3); // 2^14, 2^20

  partial_data1 = (float)rawPressure * (float)rawPressure;
  partial_data2 = ((float)(bmp->p9) / 281474976710656.0f) + ((float)(bmp->p10) / 281474976710656.0f) * bmp->temp; // 2^48, 2^48
  partial_data3 = partial_data1 * partial_data2;
  float partial_data4 = partial_data3 + ((float)rawPressure * (float)rawPressure * (float)rawPressure) * ((float)(bmp->p11) / 36893488147419103232.0f); // 2^65

  float convertedPressure = partial_out1 + partial_out2 + partial_data4;

  return convertedPressure;
}

/**
 * Write a single byte of data to a particular register on the BMP390.
 *
 * @param reg: The register address to write to.
 * @param val: The data to write to the register.
 *
 * @returns: 1 if error, 0 if no error occurred during transmission.
 */
uint8_t bmp390WriteReg(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(BMP390_I2C_HANDLE, BMP390_I2C_ADDRESS, reg, 1, &val, 1, HAL_MAX_DELAY) != HAL_OK;
}

/**
 * Reads a single byte of data from a particular register on the BMP390.
 *
 * @param reg: The register address to read from.
 * @param val: The address (in memory) of where the read value should be stored.
 *
 * @returns: 1 if error, 0 if no error occurred during transmission.
 */
uint8_t bmp390ReadReg(uint8_t reg, uint8_t* val) {
  return HAL_I2C_Mem_Read(BMP390_I2C_HANDLE, BMP390_I2C_ADDRESS, reg, 1, val, 1, HAL_MAX_DELAY) != HAL_OK;
}

/**
 * Reads multiple bytes of data starting from a register on the BMP390.
 * Note that register addresses are automatically incremented.
 *
 * @param reg: The starting register address to read from.
 * @param val: The address (in memory) of where the read values should be stored.
 *
 * @returns: 1 if error, 0 if no error occurred during transmission.
 */
uint8_t bmp390ReadRegMulti(uint8_t reg, uint8_t* vals, uint8_t size) {
  return HAL_I2C_Mem_Read(BMP390_I2C_HANDLE, BMP390_I2C_ADDRESS, reg, 1, vals, size, HAL_MAX_DELAY) != HAL_OK;
}
