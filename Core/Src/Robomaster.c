/*
 * Robomaster.c
 *
 *  Created on: Oct 1, 2024
 *      Author: saitou
 */

/*
 * RoboMaster.c
 *
 *  Created on: Sep 11, 2024
 *      Author: Tomoki Yasukawa
 */

#include "Robomaster.h"

void get_moto_offset(moto_measure_t* ptr, uint8_t RxData[])
{
    ptr->ecd        = (uint16_t)(RxData[0] << 8 | RxData[1]);
    ptr->offset_ecd = ptr->ecd;
}

void encoder_data_handler(moto_measure_t* ptr, uint8_t RxData[])
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(RxData[0] << 8 | RxData[1]);

  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  ptr->speed_rpm     = ((int16_t)RxData[2] << 8 | RxData[3]);
  ptr->given_current = ((int16_t)RxData[4] << 8 | RxData[5]);
}

float encoder_1(moto_measure_t* ptr, uint8_t RxData[])
{
	float x;
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(RxData[0] << 8 | RxData[1]);

  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  ptr->speed_rpm     = ((int16_t)RxData[2] << 8 | RxData[3]);
  ptr->given_current = ((int16_t)RxData[4] << 8 | RxData[5]);
  x = ptr->total_angle;

  return x;
}

int encoder_current(moto_measure_t* ptr, uint8_t RxData[])
{
  int m;
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(RxData[0] << 8 | RxData[1]);

  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  ptr->speed_rpm     = ((int16_t)RxData[2] << 8 | RxData[3]);
  ptr->given_current = ((int16_t)RxData[4] << 8 | RxData[5]);
  m = ptr->given_current;

  return m;
}

float encoder_rpm(moto_measure_t* ptr, uint8_t RxData[])
{
  float rpm;
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(RxData[0] << 8 | RxData[1]);

  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  ptr->speed_rpm     = ((int16_t)RxData[2] << 8 | RxData[3]);
  ptr->given_current = ((int16_t)RxData[4] << 8 | RxData[5]);
  rpm = ptr->speed_rpm;

  return rpm;
}
