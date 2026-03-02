/*
 * encoder.c
 *
 *  Created on: 2023/02/01
 *      Author: tomirobo-nuc
 */

// Encoder.c
#include "Encoder.h"

const int8_t TABLE[] = {0, -1,  1,  0, 1,  0,  0, -1, -1,  0,  0,  1, 0,  1, -1,  0  };

// Encoder initialization function
void Encoder_Init(Encoder_t* encoder) {
    // GPIO Configuration
    GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

    // Channel A Configuration
    gpio_init.Pin = encoder->channelA_pin;
    HAL_GPIO_Init(encoder->channelA_port, &gpio_init);

    // Channel B Configuration
    gpio_init.Pin = encoder->channelB_pin;
    HAL_GPIO_Init(encoder->channelB_port, &gpio_init);

    // Reset encoder position
    Encoder_ResetPosition(encoder);

    // Interrupt Configuration
    // ...
}

// Encoder deinitialization function
void Encoder_DeInit(Encoder_t* encoder) {
    // Deinitialize Channel A
    HAL_GPIO_DeInit(encoder->channelA_port, encoder->channelA_pin);

    // Deinitialize Channel B
    HAL_GPIO_DeInit(encoder->channelB_port, encoder->channelB_pin);
}

// Encoder position reset function
void Encoder_ResetPosition(Encoder_t* encoder) {
    encoder->position_deg = 0;
    encoder->position_rad = 0;
    encoder->count = 0;
    encoder->last_AB = 0;
}

int32_t Encoder_readCount(Encoder_t* encoder)
{
    uint8_t current_AB = ((HAL_GPIO_ReadPin(encoder->channelA_port, encoder->channelA_pin) << 1) | HAL_GPIO_ReadPin(encoder->channelB_port, encoder->channelB_pin));
    if (current_AB != encoder->last_AB)
    {
        encoder->count += TABLE[current_AB | (encoder->last_AB << 2)];
    }
    encoder->last_AB = current_AB;
    return encoder->count;
}

// Encoder position get function
float Encoder_GetPositionDeg(Encoder_t* encoder) {
    encoder->position_rad = encoder->count * 360.0 / encoder->resolution / 4.0;
    return encoder->position_deg;
}

// Encoder position get function
float Encoder_GetPositionRad(Encoder_t* encoder) {
    encoder->position_rad = encoder->count * 3.14159265 / encoder->resolution / 4.0;
    return encoder->position_rad;
}
