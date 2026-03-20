/*
 * encoder.c
 *
 *  Updated for TIM2 (Motor A) and TIM4 (Motor C)
 *  Wheel circumference = 20.4 cm
 *  Pulses per rev = 330, x4 encoding = 1320 ticks/rev
 *  Counter period = 65535
 */

#include "encoder.h"
#include <stdio.h>
#include "oled.h"

/* External timer handles (from CubeMX) */
extern TIM_HandleTypeDef htim2;   // Motor A encoder
extern TIM_HandleTypeDef htim4;   // Motor C encoder
#define CM_PER_TICK   (WHEEL_CIRCUMFERENCE_CM / (float)TICKS_PER_WHEEL_REV)


/* Encoder instances */
Encoder encA;
Encoder encC;

/* === Init functions === */
void encoder_init(Encoder* encoder, TIM_HandleTypeDef* htim) {
    encoder->htim = *htim;
    encoder->cnt1 = 0;
    encoder->cnt2 = 0;
    encoder->diff = 0;
    encoder->dir  = 0;
}

void start_encoder(Encoder *encoder) {
    HAL_TIM_Encoder_Start(&(encoder->htim), TIM_CHANNEL_ALL);
}

/* === Counter updates === */
void update_cnt1(Encoder *encoder) {
    encoder->cnt1 = __HAL_TIM_GET_COUNTER(&(encoder->htim));
}

void update_cnt2(Encoder *encoder) {
    encoder->cnt2 = __HAL_TIM_GET_COUNTER(&(encoder->htim));
}

/* === Compute difference with wraparound === */
void update_diff(Encoder *encoder) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&(encoder->htim))) {
        if (encoder->cnt2 < encoder->cnt1) {
            if (encoder->cnt1 - encoder->cnt2 > 2000)
                encoder->diff = 0;
            else
                encoder->diff = encoder->cnt1 - encoder->cnt2;
        } else {
            if (65535 - encoder->cnt2 + encoder->cnt1 > 2000)
                encoder->diff = 0;
            else
                encoder->diff = 65535 - encoder->cnt2 + encoder->cnt1;
        }
    } else {
        if (encoder->cnt2 > encoder->cnt1) {
            if (encoder->cnt2 - encoder->cnt1 > 2000)
                encoder->diff = 0;
            else
                encoder->diff = encoder->cnt2 - encoder->cnt1;
        } else {
            if (65535 - encoder->cnt1 + encoder->cnt2 > 2000)
                encoder->diff = 0;
            else
                encoder->diff = 65535 - encoder->cnt1 + encoder->cnt2;
        }
    }
}

/* === Direction === */
void update_direction(Encoder *encoder) {
    encoder->dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&(encoder->htim));
}

/* === OLED display helper === */
void show_diff(Encoder *encoder, uint8_t x, uint8_t y, uint8_t *buffer) {
    sprintf((char *)buffer, "S:%5ld", (long)encoder->diff);
    OLED_ShowString(x, y, buffer);
}

/* === Distance helpers === */
uint32_t calculate_target_diff(float target_distance_cm) {
    // Convert cm to ticks
    float wheel_rotations = target_distance_cm / WHEEL_CIRCUMFERENCE_CM;
    return (uint32_t)(wheel_rotations * TICKS_PER_WHEEL_REV);
}

float calculate_distance_from_ticks(uint32_t ticks) {
    float wheel_rotations = (float)ticks / TICKS_PER_WHEEL_REV;
    return wheel_rotations * WHEEL_CIRCUMFERENCE_CM;
}

void encoders_start_both(void) {
	encoder_init(&encA, &htim2);
	encoder_init(&encC, &htim4);

	HAL_TIM_Encoder_Start(&encA.htim, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&encA.htim, 0);

	HAL_TIM_Encoder_Start(&encC.htim, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&encC.htim, 0);
}


void Encoders_OLED_TestBoth(uint16_t sample_dt_ms)
{
    char l1[32], l2[32], l3[32], l4[32];

    /* Start encoders in your friend’s style (copy handle, start TIMx Encoder) */
    encoders_start_both();

    /* Trip accumulators in ticks */
    int64_t A_total_ticks = 0;
    int64_t C_total_ticks = 0;

    /* Prime previous time (ms) */
    uint32_t t_prev = HAL_GetTick();

    for (;;) {
        /* 1) First snapshot */
        update_cnt1(&encA);
        update_cnt1(&encC);

        /* 2) Wait desired sample window */
        HAL_Delay(sample_dt_ms);

        /* 3) Second snapshot + diff (wrap-safe), direction flag */
        update_cnt2(&encA);
        update_cnt2(&encC);
        update_diff(&encA);
        update_diff(&encC);
        update_direction(&encA);
        update_direction(&encC);

        /* 4) Measure *actual* elapsed time (seconds) */
        uint32_t t_now = HAL_GetTick();
        float dt = (t_now - t_prev) / 1000.0f;
        if (dt <= 0.0f) dt = (float)sample_dt_ms / 1000.0f; // fallback
        t_prev = t_now;

        /* 5) Instant distances (cm) and speeds (cm/s) */
        float dA_cm = encA.diff * CM_PER_TICK;
        float dC_cm = encC.diff * CM_PER_TICK;
        float vA_cms = dA_cm / dt;
        float vC_cms = dC_cm / dt;

        /* 6) Trip accumulation */
        A_total_ticks += encA.diff;
        C_total_ticks += encC.diff;
        float A_trip_cm = (float)A_total_ticks * CM_PER_TICK;
        float C_trip_cm = (float)C_total_ticks * CM_PER_TICK;

        /* 7) Draw to OLED (keep coords separated so text doesn’t overlap) */
        snprintf(l1, sizeof(l1), "TPR=%d", TICKS_PER_WHEEL_REV);
        snprintf(l2, sizeof(l2), "A v=%6.2f d=%6.1f", vA_cms, A_trip_cm);
        snprintf(l3, sizeof(l3), "C v=%6.2f d=%6.1f", vC_cms, C_trip_cm);
        //snprintf(l4, sizeof(l4), "dt=%ums  C=%.1f", (unsigned)sample_dt_ms, WHEEL_CIRCUMFERENCE_CM);

        OLED_Clear();
        //OLED_ShowString(10,  0, (uint8_t*)l1);
        OLED_ShowString(10, 20, (uint8_t*)l2);
        OLED_ShowString(10, 30, (uint8_t*)l3);
        //OLED_ShowString(10, 20, (uint8_t*)l4);
        OLED_Refresh_Gram();
    }
}

void EncoderA_ShowTicks(uint16_t sample_dt_ms)
{
    char buf[32];

    /* Start encoder A */
    encoder_init(&encA, &htim2);
    start_encoder(&encA);

    for (;;) {
        update_cnt1(&encA);   // snapshot current ticks

        snprintf(buf, sizeof(buf), "EncAticks=%5lu",(unsigned long)encA.cnt1);

        OLED_Clear();
        OLED_ShowString(10, 20, (uint8_t*)buf);
        OLED_Refresh_Gram();

        HAL_Delay(sample_dt_ms);
    }
}


void EncoderC_ShowTicks(uint16_t sample_dt_ms)
{
    char buf[32];

    /* Start encoder C */
    encoder_init(&encC, &htim4);
    start_encoder(&encC);

    for (;;) {
        update_cnt1(&encC);   // snapshot current ticks

        snprintf(buf, sizeof(buf), "Cticks=%5lu", (unsigned long)encC.cnt1);

        OLED_Clear();
        OLED_ShowString(10, 20, (uint8_t*)buf);
        OLED_Refresh_Gram();

        HAL_Delay(sample_dt_ms);
    }
}




