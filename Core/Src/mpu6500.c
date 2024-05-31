/*
 * mpu6500.c
 *
 *  Created on: Apr 21, 2024
 *      Author: Cody Rupp
 */

#include "stm32f4xx_hal.h"
#include "mpu6500.h"
#include <math.h>

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

  // Reset internal registers to restore default settings
  errors += mpu6500WriteReg(MPU6500_PWR_MGMT_1, 0b10000001);
  HAL_Delay(100);
  errors += mpu6500WriteReg(MPU6500_SIGNAL_PATH_RESET, 0b00000111);
  HAL_Delay(100);

  // Test that SPI communication is working as expected
  uint8_t testValue = 0;
  errors += mpu6500ReadReg(MPU6500_WHO_AM_I, &testValue);
  if (testValue != 0x70) {
    errors += 1;
  }

  // FIFO overwrites old data when full, no FSYNC,
  //  LPF settings: 10Hz cutoff frequency, 1kHz sample frequency - has 17.85 ms delay (see register map page 13/14)
  errors += mpu6500WriteReg(MPU6500_CONFIG, 0b00000101);

  // Disable gyro self-test, select scale of +/- 500 degrees per second, don't bypass LPF from config reg
  errors += mpu6500WriteReg(MPU6500_GYRO_CONFIG, 0b00001000);

  // Disable accel self-test, select scale of +/- 8g (g is force of gravity)
  errors += mpu6500WriteReg(MPU6500_ACCEL_CONFIG, 0b00010000);

  // Initialize struct fields to 0 to eliminate garbage values existing in the struct.
  mpu->roll = 0;
  mpu->pitch = 0;
  mpu->verticalVelocity = 0;
  mpu->accelOffsetX = 0;
  mpu->accelOffsetY = 0;
  mpu->accelOffsetZ = 0;
  mpu->accelerationX = 0;
  mpu->accelerationY = 0;
  mpu->accelerationZ = 0;
  mpu->angularVelocityX = 0;
  mpu->angularVelocityY = 0;
  mpu->angularVelocityZ = 0;
  mpu->gyroOffsetX = 0;
  mpu->gyroOffsetY = 0;
  mpu->gyroOffsetZ = 0;
  mpu->accelRoll = 0;
  mpu->accelPitch = 0;
  mpu->angularVPitch = 0;
  mpu->angularVRoll = 0;
  mpu->rollUncertainty = MPU6500_INITIAL_ANGLE_VARIANCE;
  mpu->pitchUncertainty = MPU6500_INITIAL_ANGLE_VARIANCE;
  // Initialize the time to the current number of ticks instead of 0 (will be more accurate)
  mpu->timeUpdated = htim2.Instance->CNT;

  // Calibrate the gyro to determine offset values
  errors += mpu6500CalibrateGyro(mpu);

  // Calibrate the accelerometer to determine offset values
  errors += mpu6500CalibrateAccel(mpu);

  return errors;
}

/**
 * Updates the acceleration and angular velocity values in a given MPU6500 struct.
 * NOTE: mpu6500Init() MUST be called prior to using this function.
 *
 * @param mpu: The address of the MPU6500 sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t mpu6500Update(MPU6500* mpu) {
  uint8_t errors = 0;

  uint32_t curTime = htim2.Instance->CNT;
  float timeDiff = (float)(curTime - mpu->timeUpdated) / US_TO_S;

  errors += mpu6500UpdateAcceleration(mpu);
  mpu6500UpdateAccelerationAngles(mpu);
  errors += mpu6500UpdateAngularVelocity(mpu);
  mpu6500UpdateAngularVelocityAngles(mpu, timeDiff);
  mpu6500UpdateKalmanAngles(mpu, timeDiff);
  mpu6500UpdateVerticalVelocity(mpu, timeDiff);

  mpu->timeUpdated = curTime;

  return errors;
}

/**
 * Updates the pitch and roll values of the IMU by applying a kalman filter using the gyroscope and pitch angles.
 * NOTE: mpu6500UpdateAccelerationAngles() should be called prior to using this function.
 * Units: Degrees.
 *
 * @param mpu: The address of the MPU6500 sensor struct.
 * @param timeDiff: The change in time (in seconds) since the last call to this function.
 */
void mpu6500UpdateKalmanAngles(MPU6500* mpu, float timeDiff) {
  // Use gyroscope values as predicted value
  mpu->pitch += timeDiff * mpu->angularVelocityX;
  mpu->roll += timeDiff * mpu->angularVelocityY;

  // Calculate uncertainty for each angle
  mpu->pitchUncertainty += (timeDiff * timeDiff) * MPU6500_GYRO_ANGLE_VARIANCE;
  mpu->rollUncertainty += (timeDiff * timeDiff) * MPU6500_GYRO_ANGLE_VARIANCE;

  float pitchKalmanGain = mpu->pitchUncertainty / (mpu->pitchUncertainty + MPU6500_ACCEL_ANGLE_VARIANCE);
  float rollKalmanGain = mpu->rollUncertainty / (mpu->rollUncertainty + MPU6500_ACCEL_ANGLE_VARIANCE);

  // This uses the gyroscope values as the "predicted angle" and accelerometer values as the "measured angle" for the kalman equations
  mpu->pitch += pitchKalmanGain * (mpu->accelPitch - mpu->pitch);
  mpu->roll += rollKalmanGain* (mpu->accelRoll - mpu->roll);

  mpu->pitchUncertainty *= (1 - pitchKalmanGain);
  mpu->rollUncertainty *= (1 - rollKalmanGain);
}

/**
 * Updates the absolute vertical velocity (opposite to direction of gravity) based on the accelerometer readings.
 * mpu6500UpdateAcceleration() and mpu6500UpdateKalmanAngles() should be called prior to using this function.
 * Units: Meters per second.
 *
 * @param mpu: The address of the MPU sensor struct to be updated.
 * @param timeDiff: The change in time (in seconds) since the last call to this function.
 */
void mpu6500UpdateVerticalVelocity(MPU6500* mpu, float timeDiff) {
  float xComponent = -1*mpu->accelerationX * sinf(mpu->pitch * M_PI / 180);
  float yComponent = mpu->accelerationY * sinf(mpu->roll * M_PI / 180) * cosf(mpu->pitch * M_PI / 180);
  float zComponent = mpu->accelerationZ * cosf(mpu->roll * M_PI / 180) * cosf(mpu->pitch * M_PI / 180);

  float verticalAcceleration = (xComponent + yComponent + zComponent - 1) * 9.81;

  mpu->verticalVelocity += timeDiff * verticalAcceleration;
}

/**
 * Reads the MPU6500's accelerometer and updates the values in an MPU struct.
 * Units: g's (Force of gravity); 1g ~= 9.81 m/(s^2)
 *
 * TODO: replace hard-coded 2048 value with value from config register
 *
 * @param mpu: The address of the MPU sensor struct to be updated.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t mpu6500UpdateAcceleration(MPU6500* mpu) {
  uint8_t errors = 0;
  uint8_t buf[2] = {0, 0}; // Temporary buffer to store raw register values
  int16_t formattedData = 0; // Stores the merged 2 bytes from the buffer before scaling the value.

  // Fill X data
  errors += mpu6500ReadReg(MPU6500_ACCEL_XOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_ACCEL_XOUT_L, buf+1);
  formattedData = (buf[0] << 8) | buf[1];
  mpu->accelerationX = ((float)formattedData / 4096) - mpu->accelOffsetX;

  // Fill Y data
  errors += mpu6500ReadReg(MPU6500_ACCEL_YOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_ACCEL_YOUT_L, buf+1);
  formattedData = (buf[0] << 8) | buf[1];
  mpu->accelerationY = ((float)formattedData / 4096) - mpu->accelOffsetY;

  // Fill Z data
  errors += mpu6500ReadReg(MPU6500_ACCEL_ZOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_ACCEL_ZOUT_L, buf+1);
  formattedData = (buf[0] << 8) | buf[1];
  mpu->accelerationZ = ((float)formattedData / 4096) - mpu->accelOffsetZ;

  return errors;
}

/**
 * Reads the MPU6500's gyroscope and updates the values in an MPU struct.
 * Units: Degrees per second.
 *
 * TODO: replace hard-coded float value with value dependent on configuration register
 *
 * @param mpu: The address of the MPU sensor struct to be updated.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t mpu6500UpdateAngularVelocity(MPU6500* mpu) {
  uint8_t errors = 0;
  uint8_t buf[2] = {0, 0}; // Temporary buffer to store raw register values
  int16_t formattedData = 0; // Stores the merged 2 bytes from the buffer before scaling the value.

  // Fill X data
  errors += mpu6500ReadReg(MPU6500_GYRO_XOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_GYRO_XOUT_L, buf+1);
  formattedData = (buf[0] << 8) | buf[1];
  mpu->angularVelocityX = ((float)formattedData / 65.5) - mpu->gyroOffsetX;

  // Fill Y data
  errors += mpu6500ReadReg(MPU6500_GYRO_YOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_GYRO_YOUT_L, buf+1);
  formattedData = (buf[0] << 8) | buf[1];
  mpu->angularVelocityY = ((float)formattedData / 65.5) - mpu->gyroOffsetY;

  // Fill Z data
  errors += mpu6500ReadReg(MPU6500_GYRO_ZOUT_H, buf);
  errors += mpu6500ReadReg(MPU6500_GYRO_ZOUT_L, buf+1);
  formattedData = (buf[0] << 8) | buf[1];
  mpu->angularVelocityZ = ((float)formattedData / 65.5) - mpu->gyroOffsetZ;

  return errors;
}

/**
 * Calculates roll and pitch angles based on the accelerometer data, and updates the values in an MPU6500 struct.
 * Units: Degrees.
 * NOTE: mpu6500UpdateAcceleration() should be called prior to using this function in order to get accurate data.
 * Depending on the fixed orientation of the IMU, the pitch and roll might need to be swapped.
 * WARNING: The accelerometer is extremely sensitive to movement, and so the angle calculated from this function alone will be quite noisy.
 *
 * @param mpu: The address of the MPU6500 sensor struct to be updated.
 */
void mpu6500UpdateAccelerationAngles(MPU6500* mpu) {
  float denominator = sqrtf(mpu->accelerationX * mpu->accelerationX + mpu->accelerationZ * mpu->accelerationZ);
  mpu->accelPitch = atan2f(mpu->accelerationY, denominator) * (180 / M_PI);

  denominator = sqrtf(mpu->accelerationY * mpu->accelerationY + mpu->accelerationZ * mpu->accelerationZ);
  mpu->accelRoll = atan2f(-1 * mpu->accelerationX, denominator) * (180 / M_PI);
}

/**
 * Calculates roll and pitch angles based on the gyroscope data, and updates the values in an MPU6500 struct.
 * Units: Degrees.
 * NOTE: mpu6500UpdateAngularVelocity() should be called prior to using this function in order to get accurate data.
 * Depending on the fixed orientation of the IMU, the pitch and roll might need to be swapped.
 * WARNING: Due to imperfections in the gyroscope readings, this function will accumulate error over time (known as gyroscope drift).
 *
 * @param mpu: The address of the MPU6500 sensor struct to be updated.
 * @param timeDiff: The change in time (in seconds) since the last call to this function.
 */
void mpu6500UpdateAngularVelocityAngles(MPU6500* mpu, float timeDiff) {
  // Integrate angular velocity to get absolute angle
  mpu->angularVPitch += mpu->angularVelocityX * timeDiff;
  mpu->angularVRoll += mpu->angularVelocityY * timeDiff;
}

/**
 * Samples the gyroscope data on startup to determine offset values, and updates the values in an MPU struct.
 * Craft should be stationary while this is happening.
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t mpu6500CalibrateGyro(MPU6500* mpu) {
  uint8_t errors = 0;

	// declare number of desired samples, arrays to store individual and total sample data
	uint16_t numSamples = 2000;
	float offsetData[] = {0, 0, 0};

	// collect samples and store the sum in offSetData array
	for(uint16_t i = 0; i < numSamples; i++) {
		errors += mpu6500UpdateAngularVelocity(mpu);

		offsetData[0] += mpu->angularVelocityX;
		offsetData[1] += mpu->angularVelocityY;
		offsetData[2] += mpu->angularVelocityZ;

		HAL_Delay(1);
	}

	// assign offset values based on the average
	mpu->gyroOffsetX = offsetData[0] / numSamples;
	mpu->gyroOffsetY = offsetData[1] / numSamples;
	mpu->gyroOffsetZ = offsetData[2] / numSamples;

	return errors;
}

/**
 * Samples the accelerometer data on startup to determine offset values, and updates the values in an MPU struct.
 * Craft should be stationary and sitting flat while this is happening.
 *
 * @param mpu: The address of the MPU sensor struct.
 *
 * @returns: The number of errors that occurred during transmission.
 */
uint8_t mpu6500CalibrateAccel(MPU6500* mpu) {
  uint8_t errors = 0;

	// declare number of desired samples, arrays to store individual and total sample data
	uint16_t numSamples = 2000;
	float offsetData[] = {0, 0, 0};

	// collect samples and store the sum in offSetData array
	for(uint16_t i = 0; i < numSamples; i++) {
		errors += mpu6500UpdateAcceleration(mpu);

		offsetData[0] += mpu->accelerationX;
		offsetData[1] += mpu->accelerationY;
		offsetData[2] += mpu->accelerationZ - 1; // Subtract 1 to account for gravity (assumes upright position)

		HAL_Delay(1);
	}

	// Get the average offset and store that in the struct.
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
