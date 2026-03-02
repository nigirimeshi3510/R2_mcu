/*
 * MDDS30_CubeIDE.h
 *
 *  Created on: 2022/12/28
 *      Author: tomirobo-nuc
 */

#ifndef MDDS30_CUBEIDE_H_
#define MDDS30_CUBEIDE_H_

#include <math.h>
#include "main.h"

void MDDS30_Serial(UART_HandleTypeDef *huart, int *ch, int *dir, double *rate);
void MDDS30_Serial_Output_Cancel(UART_HandleTypeDef *huart,int ch, int dir, double rate);
void MDDS30_Serial_Output(UART_HandleTypeDef *huart,int ch, int dir, double rate);

#endif /* MDDS30_CUBEIDE_H_ */
