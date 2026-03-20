/*
 * ultrasonic.c
 *
 *  Created on: Sep 19, 2025
 *      Author: trev0008
 */
#include "ultrasonic.h"
#include <stdio.h>
#include "cmsis_os.h"

static float echo = 0;
static uint32_t tc1, tc2;

float speedOfSound = 0.0343f / 2.0f; // cm/us

// Microsecond delay using TIM6
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim14, 0);     // reset counter
    HAL_TIM_Base_Start(&htim14);           // start timer
    while (__HAL_TIM_GET_COUNTER(&htim14) < us);
    HAL_TIM_Base_Stop(&htim14);            // optional: stop timer to save power
}

// Initialize GPIOs and timers for ultrasonic
void Ultrasonic_Init(void){
    // Configure TRIG pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = ULTRASONIC_TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ULTRASONIC_TRIG_PORT, &GPIO_InitStruct);

    // Set TRIG low initially
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);

    // Enable TIM8 input capture
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
}

// Measure distance in cm
float Ultrasonic_ReadDistance(void){
    char buf[20];

    // Send a 10us trigger pulse
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);

    if (osKernelGetState() == osKernelRunning) {
        osDelay(20);
    } else {
        HAL_Delay(20);
    }

    // Convert echo time to distance
    float distance = echo * speedOfSound;
    return distance;
}

// Input capture callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if(htim==&htim8){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET){	//If pin on high, means positive edge
			tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//Retrive value and store in tc1
		} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET){	//If pin on low means negative edge
			tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//Retrive val and store in tc2
			if (tc2 > tc1){
				echo = tc2-tc1;		//Calculate the differnce = width of pulse
			} else {
				echo = (65536-tc1)+tc2;
			}
		}

	}
}
