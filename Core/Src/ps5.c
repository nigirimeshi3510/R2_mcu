/*
 * ps5.c
 *
 *  Created on: 2023/02/01
 *      Author: Takeru Ito
 */

#include <main.h>
#include <sensor_msgs/msg/joy.h>
#include "PS5.h"

void gets_ps5(const void * msgin, PS5 *ps5)
{
	sensor_msgs__msg__Joy * msg = (sensor_msgs__msg__Joy *)msgin;

	ps5->cross_btn = msg->buttons.data[0];
	ps5->circle_btn = msg->buttons.data[1];
	ps5->triangle_btn = msg->buttons.data[2];
	ps5->square_btn = msg->buttons.data[3];

	ps5->L1_btn = msg->buttons.data[4];
	ps5->R1_btn = msg->buttons.data[5];
	ps5->L2_btn = msg->buttons.data[6];
	ps5->R2_btn = msg->buttons.data[7];

	ps5->create_btn = msg->buttons.data[8];
	ps5->select_btn = msg->buttons.data[9];

	ps5->PS_btn = msg->buttons.data[10];

	ps5->Joy_click_L = msg->buttons.data[11];
	ps5->Joy_click_R = msg->buttons.data[12];

	ps5->joy_left_x = -msg->axes.data[0];
	ps5->joy_left_y = msg->axes.data[1];

	ps5->L2 = msg->axes.data[2];

	ps5->joy_right_x = -msg->axes.data[3];
	ps5->joy_right_y = msg->axes.data[4];

	ps5->R2 = msg->axes.data[5];

	if (msg->axes.data[7] == 1)   ps5->up_btn = 1;
	if (msg->axes.data[7] == -1)  ps5->down_btn = 1;
	if (msg->axes.data[6] == 1)   ps5->left_btn = 1;
	if (msg->axes.data[6] == -1)  ps5->right_btn = 1;

	if (msg->axes.data[6] == 0) {
		ps5->left_btn = 0;
		ps5->right_btn = 0;
	}

	if (msg->axes.data[7] == 0) {
		ps5->up_btn = 0;
		ps5->down_btn = 0;
	}
}
