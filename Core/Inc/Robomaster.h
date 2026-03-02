/*
 * Robomaster.h
 *
 *  Created on: Oct 1, 2024
 *      Author: saitou
 */

#ifndef INC_ROBOMASTER_H_
#define INC_ROBOMASTER_H_

#include "main.h"

typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;

  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;

  uint16_t offset_ecd;
  uint32_t msg_cnt;

  int32_t ecd_raw_rate;
} moto_measure_t;

#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

void get_moto_offset(moto_measure_t* ptr, uint8_t RxData[]);
void encoder_data_handler(moto_measure_t* ptr, uint8_t RxData[]);
float encoder_1(moto_measure_t* ptr, uint8_t RxData[]);
int encoder_current(moto_measure_t* ptr, uint8_t RxData[]);
float encoder_rpm(moto_measure_t* ptr, uint8_t RxData[]);

#endif /* INC_ROBOMASTER_H_ */
