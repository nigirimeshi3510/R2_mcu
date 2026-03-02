/*
 * encoder.h
 *
 *  Created on: 2023/02/01
 *      Author: tomirobo-nuc
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"  // STM32F4 HAL library

// Encoder struct
typedef struct {
    GPIO_TypeDef* channelA_port;  // Channel A port
    uint16_t channelA_pin;        // Channel A pin
    GPIO_TypeDef* channelB_port;  // Channel B port
    uint16_t channelB_pin;        // Channel B pin
    float resolution;             // Encoder resolution
    int32_t count;                // Encoder count
    float position_deg;             // Encoder position
    float position_rad;             // Encoder position
    uint8_t last_AB;
} Encoder_t;

// Function prototypes
void Encoder_Init(Encoder_t* encoder);
void Encoder_DeInit(Encoder_t* encoder);
void Encoder_ResetPosition(Encoder_t* encoder);
int32_t Encoder_readCount(Encoder_t* encoder);
float Encoder_GetPositionDeg(Encoder_t* encoder);
float Encoder_GetPositionRad(Encoder_t* encoder);

#endif /* INC_ENCODER_H_ */
