/*
 * motor.h
 *
 *  Created on: Aug 30, 2025
 *      Author: Trevyen Yezdi
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

/*
 * Dual-PWM per motor (no DIR GPIO):
 *   Motor C (TIM1): FWD = CH2 (PE11), REV = CH1 (PE9)
 *   Motor A (TIM10/11): FWD = CH1 (PB9),  REV = CH1 (PB8)
 *
 * Make sure CubeMX maps these pins/channels accordingly.
 */

// Provided by CubeMX somewhere else (e.g., main.c)
extern TIM_HandleTypeDef htim1;  // Motor C: TIM1 (CH1, CH2)
extern TIM_HandleTypeDef htim10;  // Motor A: TIM4 (CH1)
extern TIM_HandleTypeDef htim11;  // Motor A: TIM4 (CH1)

/* One-time init: starts PWM on all four channels and zeros them */
void Motors_Init(void);

/* RAW COUNTS control (CCR). Values are clamped to each timer’s ARR. */
void Motors_ForwardCounts(uint32_t a_counts, uint32_t c_counts);
void Motors_ReverseCounts(uint32_t a_counts, uint32_t c_counts);

/* Convenience: set both motors the same (forward/reverse) */
void MotorsForward(uint32_t counts_same_for_both);
void MotorsReverse(uint32_t counts_same_for_both);

/* Hard stop both motors (all CCR = 0) */
void Motors_Stop(void);
void Motors_Brake(void);
void MotorA_Brake(void);
void MotorC_Brake(void);

#endif /* INC/MOTOR_H */
