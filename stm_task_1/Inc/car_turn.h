/*
 * car_turn.h
 *
 *  Created on: 18 Sep 2025
 *      Author: Geraldine Tan
 */

#ifndef INC_CAR_TURN_H_
#define INC_CAR_TURN_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========= Straight-line motion =========

/**
 * @brief Move forward to a target distance (cm) with basic yaw hold.
 *        Uses SharedSensorData for distance and yaw; brakes at the end.
 *
 * @param target_cm         Target distance to travel in cm.
 * @param base_pwm_left     Base PWM for left wheel.
 * @param base_pwm_right    Base PWM for right wheel.
 */
void Robot_MoveStraight(float target_cm,
                        uint32_t base_pwm_left, uint32_t base_pwm_right);

/**
 * @brief Move backward to a target distance (cm) with inverted yaw hold.
 *        Uses SharedSensorData for distance and yaw; brakes at the end.
 *
 * @param target_cm         Target distance to travel in cm (positive magnitude).
 * @param base_pwm_left     Base PWM for left wheel (reverse).
 * @param base_pwm_right    Base PWM for right wheel (reverse).
 */
void Robot_MoveBackward(float target_cm,
                        uint32_t base_pwm_left, uint32_t base_pwm_right);

// ========= In-place turn (blocking helper) =========

/**
 * @brief Blocking in-place turn by angleDeg using proportional control.
 *        Positive = right turn, negative = left turn. Brakes at completion.
 *
 * @param angleDeg Signed angle in degrees.
 */
void Car_Turn(float angleDeg);

// ========= Non-blocking turn (two-phase 45° + 45°) =========

/**
 * @brief Non-blocking update for a two-phase 90° turn composed of
 *        two 45° arcs at gentle PWMs (outer≈3500, inner≈3200).
 *        Advances phases when within tolerance; no overshoot logic.
 *
 *        Uses g_motion_goal:
 *          - active / started / completed
 *          - target_angle_deg (±90), turn_backward (bool)
 *          - start_yaw_deg (initialized on first entry)
 *
 *        Call periodically from your control loop/task.
 */
void Robot_TurnUpdate(void);

// ========= Combined motion =========

/**
 * @brief Move forward (PID path in your app) then flow into a turn.
 *        Currently delegates turning to Car_Turn(angleDeg).
 *
 * @param distance_cm       Distance to move before turning.
 * @param target_speed      (reserved/unused here; provided for your PID path)
 * @param Kp,Ki,Kd          (reserved/unused here; provided for your PID path)
 * @param base_pwm_left     Base PWM for left wheel during forward move.
 * @param base_pwm_right    Base PWM for right wheel during forward move.
 * @param angleDeg          Signed angle for subsequent turn (deg).
 */
void Robot_MoveAndTurnContinuous(float distance_cm,
                                 float target_speed,
                                 float Kp, float Ki, float Kd,
                                 uint32_t base_pwm_left,
                                 uint32_t base_pwm_right,
                                 float angleDeg);

#ifdef __cplusplus
}
#endif

#endif /* INC_CAR_TURN_H_ */
