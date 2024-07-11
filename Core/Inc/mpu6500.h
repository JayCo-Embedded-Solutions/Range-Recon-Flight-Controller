/*
 * MPU6500.h
 *
 *  Created on: Apr 20, 2024
 *      Author: Jeremy Lautoa-Capelle
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define MPU6500_CS_PORT GPIOB
#define MPU6500_CS_PIN  GPIO_PIN_12

extern SPI_HandleTypeDef hspi2;
#define MPU6500_SPI &hspi2

extern TIM_HandleTypeDef htim2;

#define SPI_TIMEOUT_SHORT 100
#define SPI_WRITE_BITMASK 0b01111111
#define SPI_READ_BITMASK  0b10000000

// Equal to the the standard deviation squared, which is just a guess on roughly how inaccurate each sensor angle is.
#define MPU6500_GYRO_ANGLE_VARIANCE 16
#define MPU6500_ACCEL_ANGLE_VARIANCE 9
#define MPU6500_INITIAL_ANGLE_VARIANCE 4

#define US_TO_S 1000000.0

/* Register Map */

/* Gyroscope Self-Test Registers */
#define MPU6500_SELF_TEST_X_GYRO  0x00
#define MPU6500_SELF_TEST_Y_GYRO  0x01
#define MPU6500_SELF_TEST_Z_GYRO  0x02

/* Accelerometer Self-Test Registers */
#define MPU6500_SELF_TEST_X_ACCEL 0x0D
#define MPU6500_SELF_TEST_Y_ACCEL 0x0E
#define MPU6500_SELF_TEST_Z_ACCEL 0x0F


#define MPU6500_XG_OFFSET_H     0x13
#define MPU6500_XG_OFFSET_L     0x14
#define MPU6500_YG_OFFSET_H     0x15
#define MPU6500_YG_OFFSET_L     0x16
#define MPU6500_ZG_OFFSET_H     0x17
#define MPU6500_ZG_OFFSET_L     0x18

#define MPU6500_SMPLRT_DIV      0x19
#define MPU6500_CONFIG          0x1A
#define MPU6500_GYRO_CONFIG     0x1B
#define MPU6500_ACCEL_CONFIG    0x1C
#define MPU6500_ACCEL_CONFIG_2  0x1D

#define MPU6500_LP_ACCEL_ODR    0x1E
#define MPU6500_WOM_THR         0x1F
#define MPU6500_FIFO_EN         0x23

#define MPU6500_I2C_MST_CTRL    0x24
#define MPU6500_I2C_SLV0_ADDR   0x25
#define MPU6500_I2C_SLV0_REG    0x26
#define MPU6500_I2C_SLV0_CTRL   0x27
#define MPU6500_I2C_SLV1_ADDR   0x28
#define MPU6500_I2C_SLV1_REG    0x29
#define MPU6500_I2C_SLV1_CTRL   0x2A
#define MPU6500_I2C_SLV2_ADDR   0x2B
#define MPU6500_I2C_SLV2_REG    0x2C
#define MPU6500_I2C_SLV2_CTRL   0x2D
#define MPU6500_I2C_SLV3_ADDR   0x2E
#define MPU6500_I2C_SLV3_REG    0x2F
#define MPU6500_I2C_SLV3_CTRL   0x30
#define MPU6500_I2C_SLV4_ADDR   0x31
#define MPU6500_I2C_SLV4_REG    0x32
#define MPU6500_I2C_SLV4_DO     0x33
#define MPU6500_I2C_SLV4_CTRL   0x34
#define MPU6500_I2C_SLV4_DI     0x35
#define MPU6500_I2C_MST_STATUS  0x36

#define MPU6500_INT_PIN_CFG     0x37
#define MPU6500_INT_ENABLE      0x38
#define MPU6500_INT_STATUS      0x3A

#define MPU6500_ACCEL_XOUT_H    0x3B
#define MPU6500_ACCEL_XOUT_L    0X3C
#define MPU6500_ACCEL_YOUT_H    0X3D
#define MPU6500_ACCEL_YOUT_L    0X3E
#define MPU6500_ACCEL_ZOUT_H    0X3F
#define MPU6500_ACCEL_ZOUT_L    0X40

#define MPU6500_TEMP_OUT_H      0x41
#define MPU6500_TEMP_OUT_L      0x42

#define MPU6500_GYRO_XOUT_H     0x43
#define MPU6500_GYRO_XOUT_L     0x44
#define MPU6500_GYRO_YOUT_H     0x45
#define MPU6500_GYRO_YOUT_L     0x46
#define MPU6500_GYRO_ZOUT_H     0x47
#define MPU6500_GYRO_ZOUT_L     0x48

#define MPU6500_EXT_SENS_DATA_00  0x49
#define MPU6500_EXT_SENS_DATA_01  0x4A
#define MPU6500_EXT_SENS_DATA_02  0x4B
#define MPU6500_EXT_SENS_DATA_03  0x4C
#define MPU6500_EXT_SENS_DATA_04  0x4D
#define MPU6500_EXT_SENS_DATA_05  0x4E
#define MPU6500_EXT_SENS_DATA_06  0x4F
#define MPU6500_EXT_SENS_DATA_07  0x50
#define MPU6500_EXT_SENS_DATA_08  0x51
#define MPU6500_EXT_SENS_DATA_09  0x52
#define MPU6500_EXT_SENS_DATA_10  0x53
#define MPU6500_EXT_SENS_DATA_11  0x54
#define MPU6500_EXT_SENS_DATA_12  0x55
#define MPU6500_EXT_SENS_DATA_13  0x56
#define MPU6500_EXT_SENS_DATA_14  0x57
#define MPU6500_EXT_SENS_DATA_15  0x58
#define MPU6500_EXT_SENS_DATA_16  0x59
#define MPU6500_EXT_SENS_DATA_17  0x5A
#define MPU6500_EXT_SENS_DATA_18  0x5B
#define MPU6500_EXT_SENS_DATA_19  0x5C
#define MPU6500_EXT_SENS_DATA_20  0x5D
#define MPU6500_EXT_SENS_DATA_21  0x5E
#define MPU6500_EXT_SENS_DATA_22  0x5F
#define MPU6500_EXT_SENS_DATA_23  0x60

#define MPU6500_I2C_SLV0_DO         0x63
#define MPU6500_I2C_SLV1_DO         0x64
#define MPU6500_I2C_SLV2_DO         0x65
#define MPU6500_I2C_SLV3_DO         0x66
#define MPU6500_I2C_MST_DELAY_CTRL  0x67
#define MPU6500_SIGNAL_PATH_RESET   0x68

#define MPU6500_ACCEL_INTEL_CTRL  0x69
#define MPU6500_USER_CTRL         0x6A
#define MPU6500_PWR_MGMT_1        0x6B
#define MPU6500_PWR_MGMT_2        0x6C

#define MPU6500_FIFO_COUNT_H    0x72
#define MPU6500_FIFO_COUNT_L    0x73
#define MPU6500_FIFO_R_W        0x74
#define MPU6500_WHO_AM_I        0x75

#define MPU6500_XA_OFFSET_H     0x77
#define MPU6500_XA_OFFSET_L     0x78
#define MPU6500_YA_OFFSET_H     0x7A
#define MPU6500_YA_OFFSET_L     0x7B
#define MPU6500_ZA_OFFSET_H     0x7D
#define MPU6500_ZA_OFFSET_L     0x7E

typedef struct mpu6500 {
  // Kalman-filtered angles (use these for the most accurate data)
  float roll;
  float pitch;

  // Velocity from accelerometer in the absolute vertical direction
  float verticalVelocity;
  float verticalAcceleration;

  // Kalman uncertainty values
  float rollUncertainty;
  float pitchUncertainty;

  // Gyroscope sensor values
  float angularVelocityX;
  float angularVelocityY;
  float angularVelocityZ;

  // Accelerometer sensor values
  float accelerationX;
  float accelerationY;
  float accelerationZ;

  // Gyroscope calibration values
  float gyroOffsetX;
  float gyroOffsetY;
  float gyroOffsetZ;

  // Accelerometer calibration values
  float accelOffsetX;
  float accelOffsetY;
  float accelOffsetZ;

  // Accelerometer-calculated roll and pitch angles
  float accelRoll;
  float accelPitch;

  // Gyroscope-calculated roll and pitch angles
  float angularVRoll;
  float angularVPitch;

  // Timestamp that represents when the data was last updated (in microseconds)
  uint32_t timeUpdated;
} MPU6500;

uint8_t mpu6500Init(MPU6500* mpu);
uint8_t mpu6500Update(MPU6500* mpu);
void mpu6500UpdateKalmanAngles(MPU6500* mpu, float timeDiff);
void mpu6500UpdateVerticalVelocity(MPU6500* mpu, float timeDiff);
uint8_t mpu6500UpdateAcceleration(MPU6500* mpu);
uint8_t mpu6500UpdateAngularVelocity(MPU6500* mpu);
void mpu6500UpdateAccelerationAngles(MPU6500* mpu);
void mpu6500UpdateAngularVelocityAngles(MPU6500* mpu, float timeDiff);
uint8_t mpu6500CalibrateGyro(MPU6500* mpu);
uint8_t mpu6500CalibrateAccel(MPU6500* mpu);
uint8_t mpu6500WriteReg(uint8_t reg, uint8_t val);
uint8_t mpu6500ReadReg(uint8_t reg, uint8_t* val);

#endif /* INC_MPU6500_H_ */
