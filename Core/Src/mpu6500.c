/*
 * mpu6500.c
 *
 *  Created on: Apr 21, 2024
 *      Author: Cody Rupp
 */

/**
 * Stuff to decide on TODO:
 * - spi timeout constants?
 * - return error codes for functions like mpu6500WriteReg()
 *
 * Other functions TODO:
 * - self-test for accelerometer and gyroscope (could be nice to have in init to make sure it all works)
 * - read gyroscope
 * - read accelerometer
 * - read temperature sensor (note: this might just be the temp of the mpu6500 die?)
 */

#include "stm32f4xx_hal.h"
#include "mpu6500.h"

#define MPU6500_CS_PORT GPIOB
#define MPU6500_CS_PIN  GPIO_PIN_12

extern SPI_HandleTypeDef hspi2;
#define MPU6500_SPI &hspi2 // TODO: might want to configure this in a struct instead

#define SPI_TIMEOUT_SHORT 100 // TODO: is this the best way to do this? also is this a good naming scheme?
#define SPI_WRITE_BITMASK 0b01111111
#define SPI_READ_BITMASK  0b10000000

/**
 * TODO
 */
void mpu6500Init() {
  // Run the gyroscope's self-test
//  gyroSelfTest();

  // Run the accelerometer's self-test
//  accelSelfTest();

  // TODO: read WHOAMI register to ensure the device is responsive (should return 0x70)

  // FIFO overwrites old data when full, no FSYNC, default low-pass filter settings (see register map page 13/14)
  mpu6500WriteReg(MPU6500_CONFIG, 0b00000000);

  // Disable gyro self-test, select scale of +/- 500 degrees per second, don't bypass DLPF from config reg
  mpu6500WriteReg(MPU6500_GYRO_CONFIG, 0b00001000);

  // Disable accel self-test, select scale of +/- 16g (g is force of gravity)
  mpu6500WriteReg(MPU6500_ACCEL_CONFIG, 0b00011000);

  // Bypass low-pass filter to allow for maximum 4khz output data rate
  mpu6500WriteReg(MPU6500_ACCEL_CONFIG_2, 0b00001000);

  // Enable temperature, gyroscope, and accelerometer outputs to FIFO.
  //  Note that buffering of data will occur even if data path is in standby.
  mpu6500WriteReg(MPU6500_FIFO_EN, 0b11111000);

  // INT is active high and push-pull, INT pin level held until interrupt status cleared,
  //  interrupt status cleared if any read operation is performed, FSYNC is active high,
  //  FSYNC is disabled, I2C_MASTER interface pins are in bypass mode (floating high)
  mpu6500WriteReg(MPU6500_INT_PIN_CFG, 0b00110010);

  // Disable Wake On Motion interrupt, disable FIFO overflow interrupt, enable FSYNC interrupt,
  //  enable Raw Sensor Data Ready interrupt
  mpu6500WriteReg(MPU6500_INT_ENABLE, 0b00001001);

  // Enable Digital Motion Processing features, enable FIFO operation, disable I2C master interface,
  //  don't reset any modules (unnecessary since everything just started up)
  mpu6500WriteReg(MPU6500_USER_CTRL, 0b11000000);

  // Disable sleep mode, disable cycle mode, disable gyro standby, enable temperature sensor,
  //  use best available clock source (PLL if ready, else use 20MHz internal oscillator)
  mpu6500WriteReg(MPU6500_PWR_MGMT_1, 0b00000001);

  // Disable wakeup during Accelerometer Only Low Power mode, enable accelerometer, enable gyroscope
  mpu6500WriteReg(MPU6500_PWR_MGMT_2, 0b00000000);
}

/**
 * Runs a self-test on the MPU-6500 gyroscope, indicating if there is an internal issue.
 *
 * @returns: 0 if gyro status is OK, 1 if something is wrong
 */
uint8_t gyroSelfTest() {
  // TODO
  return 0;
}

/**
 * Runs a self-test on the MPU-6500 accelerometer, indicating if there is an internal issue.
 *
 * @returns: 0 if accel status is OK, 1 if something is wrong
 */
uint8_t accelSelfTest() {
  // TODO
  return 0;
}

/**
 * Writes a single byte of data to a particular register.
 *
 * @param reg: The 7-bit register address to write to.
 * @param data: The data to write to the register
 */
void mpu6500WriteReg(uint8_t reg, uint8_t data) {
  // Pull CS pin low to begin transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);

  // Transmit write bit + register address
  uint8_t regAddr = reg & SPI_WRITE_BITMASK;
  HAL_SPI_Transmit(MPU6500_SPI, &regAddr, 1, SPI_TIMEOUT_SHORT);

  // Transmit data to write
  HAL_SPI_Transmit(MPU6500_SPI, &data, 1, SPI_TIMEOUT_SHORT);

  // Pull CS pin high to end transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);
}

/**
 * Reads a single byte of data from a particular register. TODO: make this return an error code if unsuccessful
 *
 * @param reg: The 7-bit register address to read from.
 *
 * @returns: The 8 bits of data stored in the register.
 */
uint8_t mpu6500ReadReg(uint8_t reg) {
  // Pull CS pin low to begin transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);

  // Transmit read bit + register address
  uint8_t regAddr = reg | SPI_READ_BITMASK;
  HAL_SPI_Transmit(MPU6500_SPI, &regAddr, 1, SPI_TIMEOUT_SHORT);

  // Receive data read from register. NOTE: this re-uses the regAddr buffer to save space.
  HAL_SPI_Receive(MPU6500_SPI, &regAddr, 1, SPI_TIMEOUT_SHORT);

  // Pull CS pin high to end transfer
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);

  return regAddr;
}
