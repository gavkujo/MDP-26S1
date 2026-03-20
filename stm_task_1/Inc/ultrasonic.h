/*
 * ultrasonic.h
 *
 *  Created on: Sep 19, 2025
 *      Author: trev0008
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "stm32f4xx_hal.h"

// Ports & Pins
#define ULTRASONIC_TRIG_PORT GPIOB
#define ULTRASONIC_TRIG_PIN  GPIO_PIN_15

#define ULTRASONIC_ECHO_PORT GPIOC
#define ULTRASONIC_ECHO_PIN  GPIO_PIN_7

// External variables (to be defined in ultrasonic.c)
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim8;

// Function prototypes
void Ultrasonic_Init(void);
float Ultrasonic_ReadDistance(void);
void delay_us(uint16_t us);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);



#endif /* INC_ULTRASONIC_H_ */
