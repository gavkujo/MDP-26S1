/*
 * robot_state.h
 *
 *  Created on: 28 Sep 2025
 *      Author: Geraldine Tan
 */

#ifndef INC_ROBOT_STATE_H_
#define INC_ROBOT_STATE_H_

typedef enum {
    ROBOT_STATE_IDLE,
    ROBOT_STATE_MOVING_FORWARD,
    ROBOT_STATE_MOVING_BACKWARD,
    ROBOT_STATE_TURNING_LEFT,
    ROBOT_STATE_TURNING_RIGHT,
    ROBOT_STATE_EMERGENCY_STOP,
    ROBOT_STATE_ERROR
} RobotState_t;

// Global robot state (defined once in freertos.c)
extern volatile RobotState_t current_robot_state;

#endif // ROBOT_STATE_H
