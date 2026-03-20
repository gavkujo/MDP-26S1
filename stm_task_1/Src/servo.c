/* servo.c */

#include "servo.h"

extern TIM_HandleTypeDef htim12;

// === Init ===
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    Servo_Center();
}

// === Helpers ===
void Servo_Center(void)     { Servo_SetRaw(SERVO_CENTER); }
void Servo_Left(void)       { Servo_SetRaw(SERVO_LEFT); }
void Servo_Right(void)      { Servo_SetRaw(SERVO_RIGHT); }

void Servo_SlightLeft(void)  { Servo_SetRaw(SERVO_SLIGHT_LEFT_PWM); }
void Servo_SlightRight(void) { Servo_SetRaw(SERVO_SLIGHT_RIGHT_PWM); }

void Servo_SetRaw(uint16_t value)
{
    htim12.Instance->CCR1 = value;
}
