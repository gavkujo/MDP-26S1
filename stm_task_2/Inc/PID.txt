#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target_speed;     // cm/s
    float current_speed;    // cm/s

    float error;
    float previous_error;
    float integral;

    int32_t pwm_output;     // PWM duty counts
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float target_speed);
void PID_Update(PID_Controller *pid, uint32_t base_pwm, uint32_t arr_limit);

#endif
