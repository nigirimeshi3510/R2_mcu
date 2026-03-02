/*
 * duty.h
 *
 *  Created on: 2023/02/01
 *      Author: Takeru Ito
 */

#ifndef INC_DUTY_H_
#define INC_DUTY_H_

#include <main.h>

float get_duty_speed(float Lx, float Ly);
void get_duty(float *duty, const float Lx, const float Ly, const float Rx, const float Ry);

#endif /* INC_DUTY_H_ */
