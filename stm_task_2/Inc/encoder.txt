/*
 * encoder.h
 *
 *  Created on: Aug 30, 2025
 *      Author: Geraldine Tan
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>
#include "oled.h"

/* === Mechanical constants === */
#define WHEEL_CIRCUMFERENCE_CM   (20.4f)
#define TICKS_PER_WHEEL_REV      (1550)    // 330 pulses/rev × 4
#define CM_PER_TICK   (WHEEL_CIRCUMFERENCE_CM / (float)TICKS_PER_WHEEL_REV)

// === Sampling constants for PID loops ===
#define SAMPLE_DT_MS   20                    // default sample interval in ms
#define DT_SEC         (SAMPLE_DT_MS / 1000.0f)


/* Encoder state structure */
typedef struct {
    TIM_HandleTypeDef htim;   // copy of timer handle
    uint32_t cnt1;            // previous count
    uint32_t cnt2;            // current count
    int32_t  diff;            // delta ticks per update
    uint8_t  dir;             // direction (0=fwd,1=rev)
} Encoder;

/* Global encoder objects */
extern Encoder encA;   // Motor A (TIM2)
extern Encoder encC;   // Motor C (TIM4)

/* === Core API === */
void encoder_init(Encoder* encoder, TIM_HandleTypeDef* htim);
void start_encoder(Encoder* encoder);
void update_cnt1(Encoder* encoder);
void update_cnt2(Encoder* encoder);
void update_diff(Encoder* encoder);
void update_direction(Encoder* encoder);

/* Display helper (for debugging one encoder) */
void show_diff(Encoder* encoder, uint8_t x, uint8_t y, uint8_t *buffer);

/* Distance helpers */
float calculate_distance_from_ticks(uint32_t ticks);
uint32_t calculate_target_diff(float target_distance_cm);

/* === New convenience/test functions === */
void encoders_start_both(void);                // start both encoders at once
void Encoders_OLED_TestBoth(uint16_t dt_ms);   // OLED demo: show A/C speed+distance

/* === New tick display test functions === */
void EncoderA_ShowTicks(uint16_t dt_ms);       // OLED demo: show raw ticks for encoder A
void EncoderC_ShowTicks(uint16_t dt_ms);       // OLED demo: show raw ticks for encoder C

#endif /* ENCODER_H */
