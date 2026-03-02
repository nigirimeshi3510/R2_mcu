/*
 * PS5.h
 *
 *  Created on: 2023/02/01
 *      Author: Takeru Ito
 */

#ifndef INC_PS5_H_
#define INC_PS5_H_

#include <main.h>

typedef struct {
	int up_btn;
	int down_btn;
	int right_btn;
	int left_btn;
	int square_btn;
	int circle_btn;
	int triangle_btn;
	int cross_btn;
	int L1_btn;
	int R1_btn;
	int select_btn;
	int PS_btn;
	int create_btn;
	int R2_btn;
	int L2_btn;
	int Joy_click_L;
	int Joy_click_R;

	float R2;
	float L2;
	float d_pad_x;
	float d_pad_y;
	float joy_left_x;
	float joy_left_y;
	float joy_right_x;
	float joy_right_y;

} PS5;

void gets_ps5(const void * msgin, PS5 *ps5);

#endif /* INC_PS5_H_ */
