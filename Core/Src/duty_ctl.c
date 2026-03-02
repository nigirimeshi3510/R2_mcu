/*
 * duty_ctl.c
 *
 *  Created on: 2023/02/01
 *      Author: Takeru Ito
 */

#include "duty.h"
#include <math.h>

#define DUTY_BASE   3500   //MDDS30:3500

float pi = acos(-1);

float get_duty_speed(float Lx, float Ly)
{
	float theta,numerator,denominator = 1.4;

	numerator = (float)sqrt((float)pow(Lx,2) + (float)pow(Ly,2));
	theta = asin(numerator / denominator);

	return (theta);
}

void get_duty(float *duty, const float Lx, const float Ly, const float Rx, const float Ry)
{
	float theta;
	float theta_2 = get_duty_speed(Lx, Ly);
	int i;

	if (Lx != 0) {
		theta = atan(Ly / Lx);

		duty[0] = DUTY_BASE * cos(theta - pi * 1 / 4) * sin(theta_2);
		duty[1] = DUTY_BASE * cos(theta - pi * 7 / 4) * sin(theta_2);
		duty[2] = DUTY_BASE * cos(theta - pi * 5 / 4) * sin(theta_2);
		duty[3] = DUTY_BASE * cos(theta - pi * 3 / 4) * sin(theta_2);
	}

	else if (Lx == 0) {
		duty[0] = DUTY_BASE * cos(pi * 1 / 4) * sin(theta_2);
		duty[1] = DUTY_BASE * cos(- pi * 5 / 4) * sin(theta_2);
		duty[2] = DUTY_BASE * cos(- pi * 3 / 4) * sin(theta_2);
		duty[3] = DUTY_BASE * cos(pi * 1 / 4) * sin(theta_2);

		if (Ly < 0) {
				duty[0] = -duty[0];
				duty[1] = -duty[1];
				duty[2] = -duty[2];
				duty[3] = -duty[3];
		}
	}

	if (Lx < 0) {
		duty[0] = -duty[0];
		duty[1] = -duty[1];
		duty[2] = -duty[2];
		duty[3] = -duty[3];
	}

	if (Rx != 0) {
		for (i = 0;i < 4;i++) {
			duty[i] += DUTY_BASE * Rx / 4;
		}
	}
}
