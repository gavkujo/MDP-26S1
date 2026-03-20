/*
 * car_turn.c
 *
 *  Created on: 18 Sep 2025
 *      Author: Geraldine Tan
 */

#include "car_turn.h"
#include "ICM20948.h"
#include "servo.h"
#include "motor.h"
#include <math.h>
#include "encoder.h"
#include "PID.h"
#include "shared_data.h"
#include "robot_state.h"

extern I2C_HandleTypeDef hi2c2;


void Robot_MoveStraight(float target_cm,
                        uint32_t base_pwm_left, uint32_t base_pwm_right)
{
    SharedSensorData_t data;
    (void)target_cm;

    if (!g_motion_goal.active) {
        return;
    }

    if (!g_motion_goal.started) {
        if (!SharedData_GetSensorData(&data)) {
            return;
        }

        g_motion_goal.start_distance_cm = data.distance_traveled_cm;
        g_motion_goal.start_left_ticks = data.left_encoder_ticks;
        g_motion_goal.start_right_ticks = data.right_encoder_ticks;
        g_motion_goal.start_yaw_deg = data.yaw_angle_deg;
        g_motion_goal.started = 1;
        g_motion_goal.completed = 0;
    }

    if (!SharedData_GetSensorData(&data)) {
        return;
    }

    float error_yaw = data.yaw_angle_deg - g_motion_goal.start_yaw_deg;
    if (error_yaw > 180.0f) {
        error_yaw -= 360.0f;
    } else if (error_yaw < -180.0f) {
        error_yaw += 360.0f;
    }

    if (fabsf(error_yaw) > 1.0f) {
        if (error_yaw > 0.0f) {
            Servo_SlightLeft();
        } else {
            Servo_SlightRight();
        }
    } else {
        Servo_Center();
    }

    Motors_ForwardCounts(base_pwm_left, base_pwm_right);

    float travelled = data.distance_traveled_cm - g_motion_goal.start_distance_cm;
    if (travelled < 0.0f) {
        travelled = 0.0f;
    }

    if (travelled >= g_motion_goal.target_distance_cm) {
        Motors_Brake();
        Servo_Center();
        g_motion_goal.active = 0;
        g_motion_goal.started = 0;
        g_motion_goal.completed = 1;
        g_motion_goal.target_distance_cm = 0.0f;
        current_robot_state = ROBOT_STATE_IDLE;
    }
}

//void Robot_MoveStraight_NoBrake(float target_cm,
//                        uint32_t base_pwm_left, uint32_t base_pwm_right)
//{
//    encoders_start_both();
//    int64_t A_total_ticks = 0;
//    int64_t C_total_ticks = 0;
//
//    // Capture initial yaw for straight reference
//    SharedSensorData_t sd;
//    float startYaw = 0.0f;
//    if (SharedData_GetSensorData(&sd)) {
//        startYaw = sd.yaw_angle_deg;
//    }
//    const float Kp_yaw = 30.0f;   // tune this experimentally
//
//    while (1) {
//        // Snapshot encoder counters
//        update_cnt1(&encA);
//        update_cnt1(&encC);
//        HAL_Delay(SAMPLE_DT_MS);
//        update_cnt2(&encA);
//        update_cnt2(&encC);
//        update_diff(&encA);
//        update_diff(&encC);
//
//        // Update totals (signed diffs)
//        A_total_ticks += encA.diff;
//        C_total_ticks += encC.diff;
//
//        // Compute distance travelled per wheel
//        float A_cm = calculate_distance_from_ticks((int64_t)A_total_ticks);
//        float C_cm = calculate_distance_from_ticks((int64_t)C_total_ticks);
//
//        // Use max of the two wheels for forward progress (avoids undercounting if one wheel stalls)
//        float travelled_cm = fmaxf(A_cm, C_cm);
//
//        // Yaw correction
//        if (!SharedData_GetSensorData(&sd)) {
//            continue;
//        }
//
//        float currentYaw = sd.yaw_angle_deg;
//        float error_yaw = currentYaw - startYaw;
//
//        // Wrap error to [-180, 180]
//        if (error_yaw > 180.0f)  error_yaw -= 360.0f;
//        if (error_yaw < -180.0f) error_yaw += 360.0f;
//        float heading_correction = Kp_yaw * error_yaw;
//
//        // Apply correction directly to PWM
//        int32_t pwm_left  = (int32_t)base_pwm_left  - (int32_t)heading_correction;
//        int32_t pwm_right = (int32_t)base_pwm_right + (int32_t)heading_correction;
//
//        // Clamp to safe range
//        if (pwm_left < 0) pwm_left = 0;
//        if (pwm_right < 0) pwm_right = 0;
//
//        Motors_ForwardCounts((uint32_t)pwm_left, (uint32_t)pwm_right);
//
//        // Stop if distance reached
//        if (travelled_cm >= target_cm - 4.0f) {
//            break;
//        }
//    }
//}

void Robot_MoveBackward(float target_cm,
                        uint32_t base_pwm_left, uint32_t base_pwm_right)
{
    SharedSensorData_t data;
    (void)target_cm;

    if (!g_motion_goal.active) return;

    if (!g_motion_goal.started) {
        if (!SharedData_GetSensorData(&data)) return;

        g_motion_goal.start_distance_cm = data.distance_traveled_cm;
        g_motion_goal.start_left_ticks  = data.left_encoder_ticks;
        g_motion_goal.start_right_ticks = data.right_encoder_ticks;
        g_motion_goal.start_yaw_deg     = data.yaw_angle_deg;
        g_motion_goal.started           = 1;
        g_motion_goal.completed         = 0;
    }

    if (!SharedData_GetSensorData(&data)) return;

    // yaw correction is inverted when driving backward
    float error_yaw = data.yaw_angle_deg - g_motion_goal.start_yaw_deg;
    if (error_yaw > 180.0f)  error_yaw -= 360.0f;
    if (error_yaw < -180.0f) error_yaw += 360.0f;

    if (fabsf(error_yaw) > 1.0f) {
        if (error_yaw > 0.0f) {
            // backwards needs opposite steering to keep straight
            Servo_SlightRight();
        } else {
            Servo_SlightLeft();
        }
    } else {
        Servo_Center();
    }

    // run both wheels backward at base PWM
    Motors_ReverseCounts(base_pwm_left, base_pwm_right);

    // progress is how much we've moved NEGATIVE from the start (make it positive)
    float moved = g_motion_goal.start_distance_cm - data.distance_traveled_cm;
    if (moved < 0.0f) moved = 0.0f;

    if (moved >= fabsf(g_motion_goal.target_distance_cm)) {
        Motors_Brake();
        Servo_Center();
        g_motion_goal.active             = 0;
        g_motion_goal.started            = 0;
        g_motion_goal.completed          = 1;
        g_motion_goal.target_distance_cm = 0.0f;
        current_robot_state = ROBOT_STATE_IDLE;
    }
}

void Car_Turn(float angleDeg)
{
    // ===== 0. Acquire current yaw from shared data =====
    SharedSensorData_t sd_turn;
    float startYaw = 0.0f;
    if (SharedData_GetSensorData(&sd_turn)) {
        startYaw = sd_turn.yaw_angle_deg;
    }

    // ===== 1. Record target =====
    float targetYaw = startYaw + angleDeg;
    //if (targetYaw > 180.0f)  targetYaw -= 360.0f;
    //if (targetYaw < -180.0f) targetYaw += 360.0f;

    // ===== 2. Setup steering + brakes depending on turn direction =====
    int turnLeft = (angleDeg < 0);

    if (turnLeft) {
        Servo_Left();
        HAL_Delay(400);
        MotorA_Brake(); // brake left wheel
    } else {
        Servo_Right();
        HAL_Delay(400);
        MotorC_Brake(); // brake right wheel
    }

    // ===== 3. Proportional yaw control loop =====
    const float Kp = 80.0f;          // proportional gain
    const float tol = 5.0f;          // stop tolerance in degrees
    const uint32_t MIN_SPEED = 1200; // lower bound
    const uint32_t MAX_SPEED = 4000; // upper bound

    while (1) {
        if (!SharedData_GetSensorData(&sd_turn)) {
            continue;
        }
        float currentYaw = sd_turn.yaw_angle_deg;

        float error = targetYaw - currentYaw;
        if (error > 180.0f)  error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        if (fabsf(error) < tol) {
            Motors_Stop();
            Servo_Center();
            break;
        }

        float speed = fabsf(error) * Kp;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        if (speed > MAX_SPEED) speed = MAX_SPEED;

        if (turnLeft) {
            Motors_ForwardCounts(0, (uint32_t)speed);   // right drives
        } else {
            Motors_ForwardCounts((uint32_t)speed, 0);   // left drives
        }

        HAL_Delay(10); // loop rate ~100 Hz
    }
}


void Robot_MoveAndTurnContinuous(float distance_cm,
                                 float target_speed,
                                 float Kp, float Ki, float Kd,
                                 uint32_t base_pwm_left,
                                 uint32_t base_pwm_right,
                                 float angleDeg)
{
    // Step 1: Move forward using PID (no brake at the end)

    // Step 2: Flow straight into the turn
    Car_Turn(angleDeg);
}


/* car_turn.c */
/* ... keep your existing Robot_MoveStraight() and Robot_MoveStraight_NoBrake() ... */

/* ========= NEW: Non-blocking turn, called periodically from ControlTask =========
   Uses g_motion_goal fields:
     - target_angle_deg: signed angle (+right, -left)
     - started / completed / active
     - start_yaw_deg set on first entry
*/
//void Robot_TurnUpdate(void)
//{
//    if (!g_motion_goal.active) return;
//    if (fabsf(g_motion_goal.target_angle_deg) < 0.001f) return; // not a turn goal
//
//    SharedSensorData_t sd;
//    if (!SharedData_GetSensorData(&sd)) return;
//
//    // one-time setup
//    if (!g_motion_goal.started) {
//        g_motion_goal.start_yaw_deg = sd.yaw_angle_deg;
//
//        if (g_motion_goal.target_angle_deg < 0.0f) {
//            // left turn
//            Servo_Left();
//            MotorA_Brake();                  // lock left (inner) wheel
//            current_robot_state = ROBOT_STATE_TURNING_LEFT;
//        } else {
//            // right turn
//            Servo_Right();
//            MotorC_Brake();                  // lock right (inner) wheel
//            current_robot_state = ROBOT_STATE_TURNING_RIGHT;
//        }
//        g_motion_goal.started   = 1;
//        g_motion_goal.completed = 0;
//    }
//
//    // delta from start (wrap to [-180, 180])
//    float currentYaw = sd.yaw_angle_deg;
//    float delta = currentYaw - g_motion_goal.start_yaw_deg;
//    if (delta > 180.0f)  delta -= 360.0f;
//    if (delta < -180.0f) delta += 360.0f;
//
//    /* --- FIX: reverse mapping for backward pivots ---
//       Forward:  left  = -90, right = +90
//       Backward: left  = +90, right = -90  (swap)
//    */
//    const bool turn_left = (g_motion_goal.target_angle_deg < 0.0f);
//    float target_rel;
//    if (!g_motion_goal.turn_backward) {
//        target_rel = turn_left ? -90.0f :  90.0f;
//    } else {
//        target_rel = turn_left ?  90.0f : -90.0f;  // swapped for backward
//    }
//
//    const float tol_deg          = 4.0f;     // precise
//    const float slow_window_deg  = 10.0f;    // slow near target
//    const uint32_t TURN_PWM_FAST = 4500;
//    const uint32_t TURN_PWM_SLOW = 3300;
//
//    float rem = target_rel - delta;
//
//    // done?
//    if (fabsf(rem) <= tol_deg) {
//        Motors_Stop();
//        Servo_Center();
//
//        g_motion_goal.active           = 0;
//        g_motion_goal.started          = 0;
//        g_motion_goal.completed        = 1;
//        g_motion_goal.target_angle_deg = 0.0f;
//        g_motion_goal.turn_backward    = 0;
//        current_robot_state            = ROBOT_STATE_IDLE;
//        return;
//    }
//
//    // choose speed based on remaining angle
//    uint32_t turn_pwm = (fabsf(rem) <= slow_window_deg) ? TURN_PWM_SLOW : TURN_PWM_FAST;
//
//    // drive only the outer wheel, forward OR reverse depending on turn_backward
//    if (turn_left) {
//        // LEFT pivot → outer = right wheel
//        if (g_motion_goal.turn_backward) {
//            Motors_ReverseCounts(0, turn_pwm);  // right reverse
//        } else {
//            Motors_ForwardCounts(0, turn_pwm);  // right forward
//        }
//    } else {
//        // RIGHT pivot → outer = left wheel
//        if (g_motion_goal.turn_backward) {
//            Motors_ReverseCounts(turn_pwm, 0);  // left reverse
//        } else {
//            Motors_ForwardCounts(turn_pwm, 0);  // left forward
//        }
//    }
//}
//
//
// ---------- helpers ----------
static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Small state for the two-phase turn
typedef struct {
    uint8_t  in_use;
    uint8_t  phase;          // 0 = first 45°, 1 = second 45°
    float    tgt_phase0_deg; // relative to start yaw (+/-45)
    float    tgt_phase1_deg; // relative to start yaw (+/-90 total)
} TurnCtx_t;

static TurnCtx_t s_turn = {0};

// Forward mapping:
// - Left commands are negative, right commands positive.
// - If turning "backward", left/right sense flips.
// Return true if net desired yaw is to the right (positive).
static bool net_turn_right(bool cmd_left_sign_is_negative, bool backward) {
    bool is_left = cmd_left_sign_is_negative ^ backward;
    return !is_left;
}

// Differential drive at fixed slow power (no braking).
// turn_right = desired yaw direction; forward = drive direction.
static void drive_diff(bool turn_right, bool forward,
                       uint32_t pwm_outer, uint32_t pwm_inner)
{
    // NOTE: Motors_*Counts(left, right)
    if (forward) {
        if (turn_right) {
            // right yaw, forward → left=outer
            Motors_ForwardCounts(pwm_outer, pwm_inner);
        } else {
            // left yaw, forward → right=outer
            Motors_ForwardCounts(pwm_inner, pwm_outer);
        }
    } else {
        if (turn_right) {
            // right yaw, backward → right=outer
            Motors_ReverseCounts(pwm_inner, pwm_outer);
        } else {
            // left yaw, backward → left=outer
            Motors_ReverseCounts(pwm_outer, pwm_inner);
        }
    }
}

//void Robot_TurnUpdate(void)
//{
//    if (!g_motion_goal.active) return;
//    if (fabsf(g_motion_goal.target_angle_deg) < 0.001f) return; // not a turn goal
//
//    SharedSensorData_t sd;
//    if (!SharedData_GetSensorData(&sd)) return;
//
//    // --- very gentle, constant PWMs for observation ---
//    const uint32_t PWM_OUTER = 2950;   // outer wheel
//    const uint32_t PWM_INNER = 2650;   // inner wheel (slightly less)
//    const float    TOL_DEG   = 1.0f;   // phase completion tolerance
//
//    // one-time setup
//    if (!g_motion_goal.started) {
//        g_motion_goal.start_yaw_deg = sd.yaw_angle_deg;
//
//        // Decide net sign of final 90° (accounting for backward flip)
//        const bool cmd_left_is_negative = (g_motion_goal.target_angle_deg < 0.0f);
//        const bool backward             = (g_motion_goal.turn_backward != 0);
//        const bool net_right            = net_turn_right(cmd_left_is_negative, backward);
//
//        const float total_abs = fminf(fabsf(g_motion_goal.target_angle_deg), 90.0f);
//        const float total_deg = net_right ? +total_abs : -total_abs; // clamp to ±90
//
//        // Phase targets are both relative to the same start_yaw:
//        s_turn.tgt_phase0_deg = (total_deg > 0.0f ? +45.0f : -45.0f);
//        s_turn.tgt_phase1_deg = (total_deg > 0.0f ? +90.0f : -90.0f);
//        s_turn.phase          = 0;
//        s_turn.in_use         = 1;
//
//        // Steering for phase 0
//        if (net_right ^ backward) Servo_Right(); else Servo_Left();
//        current_robot_state     = net_right ? ROBOT_STATE_TURNING_RIGHT : ROBOT_STATE_TURNING_LEFT;
//
//        g_motion_goal.started   = 1;
//        g_motion_goal.completed = 0;
//    }
//
//    // Current progress
//    const float currentYaw = sd.yaw_angle_deg;
//    const float delta      = wrap180(currentYaw - g_motion_goal.start_yaw_deg);
//
//    // Pick current phase target (relative to start yaw)
//    const float tgt_rel = (s_turn.phase == 0) ? s_turn.tgt_phase0_deg : s_turn.tgt_phase1_deg;
//    const float rem     = tgt_rel - delta;  // remaining angle for this phase
//
//    // Phase completion (NO overshoot logic; just tolerance)
//    if (fabsf(rem) <= TOL_DEG) {
//        if (s_turn.phase == 0) {
//            // switch to phase 1: opposite arc at same slow power
//            s_turn.phase = 1;
//            osDelay(1000);
//            const bool cmd_left_is_negative = (g_motion_goal.target_angle_deg < 0.0f);
//            const bool backward             = (g_motion_goal.turn_backward != 0);
//            const bool net_right            = net_turn_right(cmd_left_is_negative, backward);
//
//            // Opposite steering in phase 1
//            if (net_right ^ backward) Servo_Left(); else Servo_Right();
//
//        } else {
//            // Finished second phase → stop & clean up
//        	Motors_Brake();
//            Motors_Stop();
//            Servo_Center();
//
//            s_turn.in_use = 0;
//            g_motion_goal.active           = 0;
//            g_motion_goal.started          = 0;
//            g_motion_goal.completed        = 1;
//            g_motion_goal.target_angle_deg = 0.0f;
//            g_motion_goal.turn_backward    = 0;
//            current_robot_state            = ROBOT_STATE_IDLE;
//            return;
//        }
//    }
//
//    // Drive plan for each phase (still slow)
//    const bool cmd_left_is_negative = (g_motion_goal.target_angle_deg < 0.0f);
//    const bool backward             = (g_motion_goal.turn_backward != 0);
//    const bool net_right            = net_turn_right(cmd_left_is_negative, backward);
//
//    bool phase_turn_right, phase_forward;
//
//    if (s_turn.phase == 0) {
//        // first 45° arc
//        phase_turn_right = net_right;
//        phase_forward    = !backward;     // start forward unless command is a "back" turn
//    } else {
//        // second 45° arc is the opposite yaw direction AND opposite drive direction
//        phase_turn_right = !net_right;
//        phase_forward    = backward;      // opposite of phase 0
//    }
//
//    drive_diff(phase_turn_right, phase_forward, PWM_OUTER, PWM_INNER);
//}

//static TurnCtx_t s_turn = {0};
static float s_prev_rem = 0.0f;
static bool  s_prev_rem_valid = false;
#define SCALE 0.95f
#define PHASE0 (45.0f*SCALE)
#define PHASE1 (90.0f*SCALE)
void Robot_TurnUpdate(void)
{
    if (!g_motion_goal.active) return;
    if (fabsf(g_motion_goal.target_angle_deg) < 0.001f) return;

    SharedSensorData_t sd;
    if (!SharedData_GetSensorData(&sd)) return;

    const uint32_t PWM_OUTER = 2950;
    const uint32_t PWM_INNER = 2650;
    const uint32_t PWM_BOOST = 100; //cz one of the motors weak ash
    const float    TOL_DEG   = 1.0f;
    uint32_t pwm_outer = PWM_OUTER;
    uint32_t pwm_inner = PWM_INNER;

    const bool cmd_left_is_negative = (g_motion_goal.target_angle_deg < 0.0f);
    const bool backward             = (g_motion_goal.turn_backward != 0);
    const bool net_right            = net_turn_right(cmd_left_is_negative, backward);

    if (net_right){
    	if(s_turn.phase == 1){
    		pwm_outer += PWM_BOOST;
    		pwm_inner += PWM_BOOST;
    	}
    } else {
    	if(s_turn.phase == 0){
    	    pwm_outer += PWM_BOOST;
    	    pwm_inner += PWM_BOOST;
    	}
    }

    if (!g_motion_goal.started) {
        g_motion_goal.start_yaw_deg = sd.yaw_angle_deg;

        const float total_abs = fminf(fabsf(g_motion_goal.target_angle_deg), 90.0f);
        const float total_deg = net_right ? +total_abs : -total_abs;

        s_turn.tgt_phase0_deg = (total_deg > 0.0f ? +PHASE0 : -PHASE0);
        s_turn.tgt_phase1_deg = (total_deg > 0.0f ? +PHASE1 : -PHASE1);
        s_turn.phase          = 0;
        s_turn.in_use         = 1;
        s_prev_rem_valid      = false;

        if (net_right ^ backward) Servo_Right(); else Servo_Left();
        current_robot_state = net_right ? ROBOT_STATE_TURNING_RIGHT : ROBOT_STATE_TURNING_LEFT;

        g_motion_goal.started   = 1;
        g_motion_goal.completed = 0;
    }

    const float currentYaw = sd.yaw_angle_deg;
    const float delta      = wrap180(currentYaw - g_motion_goal.start_yaw_deg);

    const float tgt_rel = (s_turn.phase == 0) ? s_turn.tgt_phase0_deg : s_turn.tgt_phase1_deg;
    const float rem     = tgt_rel - delta;

    const bool crossed = s_prev_rem_valid &&
                         ((rem > 0.0f && s_prev_rem < 0.0f) ||
                          (rem < 0.0f && s_prev_rem > 0.0f));

    if (fabsf(rem) <= TOL_DEG || crossed) {
        Motors_Brake();
        Motors_Stop();
        Servo_Center();
        s_prev_rem_valid = false;

        if (s_turn.phase == 0) {
            s_turn.phase = 1;
            osDelay(1000);

            if (net_right ^ backward) Servo_Left(); else Servo_Right();
            return;
        } else {
            s_turn.in_use = 0;
            g_motion_goal.active           = 0;
            g_motion_goal.started          = 0;
            g_motion_goal.completed        = 1;
            g_motion_goal.target_angle_deg = 0.0f;
            g_motion_goal.turn_backward    = 0;
            current_robot_state            = ROBOT_STATE_IDLE;
            return;
        }
    }

    bool phase_turn_right, phase_forward;

    if (s_turn.phase == 0) {
        phase_turn_right = net_right;
        phase_forward    = !backward;
    } else {
        phase_turn_right = !net_right;
        phase_forward    = backward;
    }

    drive_diff(phase_turn_right, phase_forward, pwm_outer, pwm_inner);

    s_prev_rem = rem;
    s_prev_rem_valid = true;
}
