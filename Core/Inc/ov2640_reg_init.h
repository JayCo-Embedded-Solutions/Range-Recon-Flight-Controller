/*
 * ov2640_reg_init.h
 *
 *  Created on: May 12, 2024
 *      Author: Cody Rupp
 */
#include <stdint.h>

#ifndef INC_OV2640_REG_INIT_H_
#define INC_OV2640_REG_INIT_H_

typedef struct sensor_reg {
  uint8_t reg;
  uint8_t val;
} sensor_reg_t;

extern const sensor_reg_t OV2640_JPEG_INIT[];
extern const sensor_reg_t OV2640_YUV422[];
extern const sensor_reg_t OV2640_JPEG[];
extern const sensor_reg_t OV2640_160x120_JPEG[];
extern const sensor_reg_t OV2640_320x240_JPEG[];

#endif /* INC_OV2640_REG_INIT_H_ */
