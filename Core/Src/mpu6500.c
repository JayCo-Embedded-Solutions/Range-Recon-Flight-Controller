/*
 * mpu6500.c
 *
 *  Created on: Apr 21, 2024
 *      Author: Cody Rupp
 */

#include "stm32f4xx_hal.h"
#include "mpu6500.h"

/**
 * Initializes the MPU6500's registers and calibrates the gyroscope and accelerometer.
 * This function MUST be called prior to reading accelerometer or gyroscope values.
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t mpu6500Init(MPU6500* mpu) {
  uint8_t errors = 0;

  // FIFO overwrites old data when full, no FSYNC, default low-pass filter settings (see register map page 13/14)
  errors += mpu6500WriteReg(MPU6500_CONFIG, 0b00000000);

  // Disable gyro self-test, select scale of +/- 500 degrees per second, don't bypass DLPF from config reg
  errors += mpu6500WriteReg(MPU6500_GYRO_CONFIG, 0b00001000);

  // Disable accel self-test, select scale of +/- 16g (g is force of gravity)
  errors += mpu6500WriteReg(MPU6500_ACCEL_CONFIG, 0b00011000);

  // Bypass low-pass filter to allow for maximum 4khz output data rate
  errors += mpu6500WriteReg(MPU6500_ACCEL_CONFIG_2, 0b00001000);

  // Enable temperature, gyroscope, and accelerometer outputs to FIFO.
  //  Note that buffering of data will occur even if data path is in standby.
  errors += mpu6500WriteReg(MPU6500_FIFO_EN, 0b11111000);

  // INT is active high and push-pull, INT pin level held until interrupt status cleared,
  //  interrupt status cleared if any read operation is performed, FSYNC is active high,
  //  FSYNC is disabled, I2C_MASTER interface pins are in bypass mode (floating high)
  errors += mpu6500WriteReg(MPU6500_INT_PIN_CFG, 0b00110010);

  // Disable Wake On Motion interrupt, disable FIFO overflow interrupt, enable FSYNC interrupt,
  //  enable Raw Sensor Data Ready interrupt
  errors += mpu6500WriteReg(MPU6500_INT_ENABLE, 0b00001001);

  // Enable Digital Motion Processing features, enable FIFO operation, disable I2C master interface,
  //  don't reset any modules (unnecessary since everything just started up)
  errors += mpu6500WriteReg(MPU6500_USER_CTRL, 0b11000000);

  // Disable sleep mode, disable cycle mode, disable gyro standby, enable temperature sensor,
  //  use best available clock source (PLL if ready, else use 20MHz internal oscillator)
  errors += mpu6500WriteReg(MPU6500_PWR_MGMT_1, 0b00000001);

  // Disable wakeup during Accelerometer Only Low Power mode, enable accelerometer, enable gyroscope
  errors += mpu6500WriteReg(MPU6500_PWR_MGMT_2, 0b00000000);

  HAL_Delay(1000);

  // Calibrate the gyro to determine offset values
  errors += calibrateGyro(mpu);

  // Calibrate the accelerometer to determine offset values
  errors += calibrateAccel(mpu);

  return errors;
}

/**
 * Reads the MPU6500's accelerometer and updates the values in an MPU struct.
 * Units: Meters per second squared.
 *
 * TODO: replace hard-coded 2048 value with value from config register
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t updateAcceleration(MPU6500* mpu) {
  uint8_t errors = 0;
  uint8_t buf[2] = {0, 0}; // Temporary buffer to store raw register values

  // Fill X data
  errors += mpu6500ReadReg(MPU6500_ACCEL_XOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_ACCEL_XOUT_L, buf+1);
  mpu->accelerationX = (int16_t)((buf[0] << 8) | buf[1]) / 2048.0f;

  // Fill Y data
  errors += mpu6500ReadReg(MPU6500_ACCEL_YOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_ACCEL_YOUT_L, buf+1);
  mpu->accelerationY = (int16_t)((buf[0] << 8) | buf[1]) / 2048.0f;

  // Fill Z data
  errors += mpu6500ReadReg(MPU6500_ACCEL_ZOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_ACCEL_ZOUT_L, buf+1);
  mpu->accelerationZ = (int16_t)((buf[0] << 8) | buf[1]) / 2048.0f;

  return errors;
}

/**
 * Reads the MPU6500's gyroscope and updates the values in an MPU struct.
 * Units: Degrees per second.
 *
 * TODO: replace hard-coded float value with value dependent on configuration register
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t updateAngularVelocity(MPU6500* mpu) {
  uint8_t errors = 0;
  uint8_t buf[2] = {0, 0}; // Temporary buffer to store raw register values

  // Fill X data
  errors += mpu6500ReadReg(MPU6500_GYRO_XOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_GYRO_XOUT_L, buf+1);
  mpu->angularVelocityX = (int16_t)((buf[0] << 8) | buf[1]) / 131.0f;

  // Fill Y data
  errors += mpu6500ReadReg(MPU6500_GYRO_YOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_GYRO_YOUT_L, buf+1);
  mpu->angularVelocityY = (int16_t)((buf[0] << 8) | buf[1]) / 131.0f;

  // Fill Z data
  errors += mpu6500ReadReg(MPU6500_GYRO_ZOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_GYRO_ZOUT_L, buf+1);
  mpu->angularVelocityZ = (int16_t)((buf[0] << 8) | buf[1]) / 131.0f;

  return errors;
}

/**
 * Samples the gyroscope data on startup to determine offset values, and updates the values in an MPU struct.
 * Craft should be stationary while this is happening.
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t calibrateGyro(MPU6500* mpu) {
  uint8_t errors = 0;

	// declare number of desired samples, arrays to store individual and total sample data
	uint8_t numSamples = 30;
	float offsetData[] = {0, 0, 0};

	// collect samples and store the sum in offSetData array
	for(uint8_t i = 0; i<numSamples; i++) {
		errors += updateAngularVelocity(mpu);

		offsetData[0] += mpu->angularVelocityX;
		offsetData[1] += mpu->angularVelocityY;
		offsetData[2] += mpu->angularVelocityZ;
	}

	// assign offset values based on the average
	mpu->gyroOffsetX = offsetData[0] / numSamples;
	mpu->gyroOffsetY = offsetData[1] / numSamples;
	mpu->gyroOffsetZ = offsetData[2] / numSamples;

	return errors;
}

/**
 * Samples the accelerometer data on startup to determine offset values, and updates the values in an MPU struct.
 * Craft should be stationary while this is happening.
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t calibrateAccel(MPU6500* mpu) {
  uint8_t errors = 0;

	// declare number of desired samples, arrays to store individual and total sample data
	uint8_t numSamples = 30;
	float offsetData[] = {0, 0, 0};

	// collect samples and store the sum in offSetData array
	for(uint8_t i = 0; i<numSamples; i++) {
		errors += updateAcceleration(mpu);

		offsetData[0] += mpu->accelerationX;
		offsetData[1] += mpu->accelerationY;
		offsetData[2] += mpu->accelerationZ - 9.8;		// subtract 9.8 for grav constant on Z axis
	}

	// collect samples and store the sum in offSetData array
	mpu->accelOffsetX = offsetData[0] / numSamples;
	mpu->accelOffsetY = offsetData[1] / numSamples;
	mpu->accelOffsetZ = offsetData[2] / numSamples;

	return errors;
}

/**
 * Writes a single byte of data to a particular register.
 *
 * @param reg: The 7-bit register address to write to.
 * @param val: The data to write to the register
 *
 * @returns: 1 if error, 0 if no error occured during transmission.
 */
uint8_t mpu6500WriteReg(uint8_t reg, uint8_t val) {
  // Pull CS pin low to begin transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);

  // Transmit write bit + register address
  uint8_t regAddr = reg & SPI_WRITE_BITMASK;
  if (HAL_SPI_Transmit(MPU6500_SPI, &regAddr, 1, SPI_TIMEOUT_SHORT) != HAL_OK) {
    return 1;
  }

  // Transmit data to write
  if (HAL_SPI_Transmit(MPU6500_SPI, &val, 1, SPI_TIMEOUT_SHORT) != HAL_OK) {
    return 1;
  }

  // Pull CS pin high to end transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);

  return 0;
}

/**
 * Reads a single byte of data from a particular register.
 *
 * @param reg: The 7-bit register address to read from.
 * @param val: The address (in memory) of where the read value should be stored.
 *
 * @returns: 1 if error, 0 if no error occurred during transmission.
 */
uint8_t mpu6500ReadReg(uint8_t reg, uint8_t* val) {
  // Pull CS pin low to begin transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);

  // Transmit read bit + register address
  uint8_t regAddr = reg | SPI_READ_BITMASK;
  if (HAL_SPI_Transmit(MPU6500_SPI, &regAddr, 1, SPI_TIMEOUT_SHORT) != HAL_OK) {
    return 1;
  }

  // Receive data read from register
  if (HAL_SPI_Receive(MPU6500_SPI, val, 1, SPI_TIMEOUT_SHORT) != HAL_OK) {
    return 1;
  }

  // Pull CS pin high to end transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);

  return 0;
}
