/*
 * MDDS30_CubeIDE.c
 *
 *  Created on: 2022/12/28
 *      Author: Takeru Ito
 */

#include "MDDS30_CubeIDE.h"

void MDDS30_Serial(UART_HandleTypeDef *huart, int *ch, int *dir, double *rate)
{
	for (int i = 0;i < 2;i++) {
			  if (rate[i] == 0)
			  	  MDDS30_Serial_Output_Cancel(huart, ch[i], dir[i], rate[i]);
			  else
				  MDDS30_Serial_Output(huart, ch[i], dir[i], rate[i]);
		  }
}

void MDDS30_Serial_Output_Cancel(UART_HandleTypeDef *huart, int ch, int dir, double rate)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   //debug LED ON
	uint8_t buff = 0;

	rate = 0;

	buff += ((ch % 2) << 7);                       //motor driver channel
	buff += (dir << 6);                            //motor driver direction
	buff += (0x3F & (uint8_t)(fabs(rate) * 63.0)); //motor driver speed

	HAL_UART_Transmit(huart, &buff, sizeof(buff), 0xFFFF);
	HAL_Delay(100);
}

void MDDS30_Serial_Output(UART_HandleTypeDef *huart,int ch, int dir, double rate)
{
	if (rate == 0)
		MDDS30_Serial_Output_Cancel(huart, ch, dir, rate);

	else {
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   //debug LED ON
	uint8_t buff = 0;

	buff += ((ch % 2) << 7);                       //motor driver channel
	buff += (dir << 6);                            //motor driver direction
	buff += (0x3F & (uint8_t)(fabs(rate) * 63.0)); //motor driver speed

	HAL_UART_Transmit(huart, &buff, sizeof(buff), 0xFFFF);
	HAL_Delay(100);
	}
}
