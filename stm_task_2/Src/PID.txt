#include "PID.h"
#include "motor.h"
#include "encoder.h"
#include "stm32f4xx_hal.h"  // HAL_Delay, timers

extern Encoder encA;
extern Encoder encC;
extern TIM_HandleTypeDef htim1;   // Motor C PWM
extern TIM_HandleTypeDef htim11;  // Motor A PWM

#define DT_SEC        (SAMPLE_DT_MS / 1000.0f)
#define CM_PER_TICK   (WHEEL_CIRCUMFERENCE_CM / (float)TICKS_PER_WHEEL_REV)

/* ===== Init ===== */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float target_speed) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target_speed = target_speed;
    pid->current_speed = 0.0f;

    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;

    pid->pwm_output = 0;
}

/* ===== Update (speed control) ===== */
void PID_Update(PID_Controller *pid, uint32_t base_pwm, uint32_t arr_limit) {
    // Error in speed
    pid->error = pid->target_speed - pid->current_speed;

    // Integral term
    pid->integral += pid->error * DT_SEC;

    // Derivative term
    float derivative = (pid->error - pid->previous_error) / DT_SEC;

    // PID output
    float output = (pid->Kp * pid->error) +
                   (pid->Ki * pid->integral) +
                   (pid->Kd * derivative);

    // Apply around base
    int32_t pwm_val = (int32_t)base_pwm + (int32_t)output;

    // Clamp
    if (pwm_val < 0) pwm_val = 0;
    if (pwm_val > (int32_t)arr_limit) pwm_val = arr_limit;

    pid->pwm_output = pwm_val;

    // Store for next loop
    pid->previous_error = pid->error;
}
