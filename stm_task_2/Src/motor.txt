/*
 * motor.c
 *
 *  Updated on: Aug 30, 2025
 *  Author: Geraldine Tan
 */

#include "motor.h"

/* ===== Channel mapping =====
 * Motor C (TIM1): REV = CH1, FWD = CH2   (PE9/PE11)
 * Motor A (TIM10 + TIM11):
 *   TIM10_CH1 (PB8, user label A_IN2) -> Motor A reverse
 *   TIM11_CH1 (PB9, user label A_IN1) -> Motor A forward
 */

static uint32_t clamp_counts(TIM_HandleTypeDef *htim, uint32_t counts) {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    return (counts > arr) ? arr : counts;
}

/* === Init === */
void Motors_Init(void)
{
    // Start all PWM channels once
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_1); // C reverse
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_2); // C forward
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // A reverse
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); // A forward

    // Zero them
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

/* === Forward === */
void Motors_ForwardCounts(uint32_t a_counts, uint32_t c_counts)
{
    a_counts = clamp_counts(&htim11, a_counts);
    c_counts = clamp_counts(&htim1,  c_counts);

    // Motor A: FWD=TIM11_CH1, ensure REV=TIM10_CH1 off
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, a_counts);

    // Motor C: FWD=CH2, ensure REV=CH1 off
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, c_counts);
}

/* === Reverse === */
void Motors_ReverseCounts(uint32_t a_counts, uint32_t c_counts)
{
    a_counts = clamp_counts(&htim10, a_counts);
    c_counts = clamp_counts(&htim1,  c_counts);

    // Motor A: REV=TIM10_CH1, ensure FWD=TIM11_CH1 off
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, a_counts);

    // Motor C: REV=CH1, ensure FWD=CH2 off
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, c_counts);
}

/* === Helpers for both wheels === */
void MotorsForward(uint32_t counts_same_for_both)
{
    Motors_ForwardCounts(counts_same_for_both, counts_same_for_both);
}

void MotorsReverse(uint32_t counts_same_for_both)
{
    Motors_ReverseCounts(counts_same_for_both, counts_same_for_both);
}

void Motors_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}


void Motors_Brake(void)
{
    // Force both motor channels ON simultaneously (shorting the motor)
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1));
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim1));
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim10));
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim11));
}

void MotorA_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim10));
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim11));
}

void MotorC_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim1));
}
