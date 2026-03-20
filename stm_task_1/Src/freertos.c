/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_priorities.h"
#include "command_parser.h"
#include "shared_data.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "ICM20948.h"
// #include "ir_sensor.h"
#include "oled.h"
#include "car_turn.h"
#include <stdio.h>
#include <string.h>
#include "robot_state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 10
#define COMMAND_TIMEOUT_MS  5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// FreeRTOS Objects
osThreadId_t CommunicationTaskHandle;
osThreadId_t ControlTaskHandle;
osThreadId_t MotorTaskHandle;
osThreadId_t OLEDTaskHandle;
osThreadId_t IMUTaskHandle;
osThreadId_t EncoderTaskHandle;

osMessageQueueId_t xCommandQueue;
osMessageQueueId_t xMotorQueue;

osSemaphoreId_t xIMUSem;
osSemaphoreId_t xEncoderSem;

osTimerId_t xUltrasonicTimer;
osTimerId_t xEncoderTimer;
osTimerId_t xIMUTimer;
// osTimerId_t xIRTimer;

// UART receive buffer
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_cmd_buffer[7]; // 6 chars + null terminator
volatile uint8_t uart_rx_index = 0;

// Robot state
volatile RobotState_t current_robot_state = ROBOT_STATE_IDLE;
volatile bool command_in_progress = false;
static volatile bool s_ultrasonic_sample_pending = false;

// External UART handle
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c2;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void CommunicationTask(void *argument);
void ControlTask(void *argument);
void MotorTask(void *argument);
void OLEDTask(void *argument);
void IMUTask(void *argument);
void EncoderTask(void *argument);
void IMUTimerCallback(void *argument);
void EncoderTimerCallback(void *argument);
void UltrasonicTimerCallback(void *argument);
// void IRTimerCallback(void *argument);

void ProcessRobotCommand(RobotCommand_t* cmd);
void SendACK(const char* message);

void StartSensorTimers(void);
void HandleEmergencyStop(void);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xIMUSem = osSemaphoreNew(1U, 0U, NULL);
  xEncoderSem = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* Create software timers */
  xUltrasonicTimer = osTimerNew(UltrasonicTimerCallback, osTimerPeriodic, NULL, NULL);
  xEncoderTimer = osTimerNew(EncoderTimerCallback, osTimerPeriodic, NULL, NULL);
  xIMUTimer = osTimerNew(IMUTimerCallback, osTimerPeriodic, NULL, NULL);
  // xIRTimer = osTimerNew(IRTimerCallback, osTimerPeriodic, NULL, NULL);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Create queues */
  xCommandQueue = osMessageQueueNew(COMMAND_QUEUE_SIZE, sizeof(RobotCommand_t), NULL);
  xMotorQueue = osMessageQueueNew(MOTOR_QUEUE_SIZE, sizeof(MotorCommand_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t CommunicationTask_attributes = {
    .name = "CommunicationTask",
    .stack_size = COMMUNICATION_TASK_STACK_SIZE * 4,
    .priority = COMMUNICATION_TASK_PRIORITY,
  };
  CommunicationTaskHandle = osThreadNew(CommunicationTask, NULL, &CommunicationTask_attributes);

  const osThreadAttr_t OLEDTask_attributes = {
      .name = "OLEDTask",
      .stack_size = OLED_TASK_STACK_SIZE * 4,
      .priority = OLED_TASK_PRIORITY,
    };
  OLEDTaskHandle = osThreadNew(OLEDTask, NULL, &OLEDTask_attributes);

  const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask",
    .stack_size = CONTROL_TASK_STACK_SIZE * 4,
    .priority = CONTROL_TASK_PRIORITY,
  };
  ControlTaskHandle = osThreadNew(ControlTask, NULL, &ControlTask_attributes);

  const osThreadAttr_t MotorTask_attributes = {
    .name = "MotorTask",
    .stack_size = MOTOR_TASK_STACK_SIZE * 4,
    .priority = MOTOR_TASK_PRIORITY,
  };
  MotorTaskHandle = osThreadNew(MotorTask, NULL, &MotorTask_attributes);

  const osThreadAttr_t IMUTask_attributes = {
    .name = "IMUTask",
    .stack_size = SENSOR_TASK_STACK_SIZE * 4,
    .priority = SENSOR_TASK_PRIORITY,
  };
  IMUTaskHandle = osThreadNew(IMUTask, NULL, &IMUTask_attributes);

  const osThreadAttr_t EncoderTask_attributes = {
    .name = "EncoderTask",
    .stack_size = SENSOR_TASK_STACK_SIZE * 4,
    .priority = SENSOR_TASK_PRIORITY,
  };
  EncoderTaskHandle = osThreadNew(EncoderTask, NULL, &EncoderTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_CommunicationTask */
/**
* @brief Function implementing the CommunicationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommunicationTask */
void CommunicationTask(void *argument)
{
  /* USER CODE BEGIN CommunicationTask */
  RobotCommand_t parsed_command;
  osStatus_t status;

  // Initialize UART for interrupt-based reception
  HAL_UART_Receive_IT(&huart3, &uart_rx_buffer[0], 1);

  /* Infinite loop */
  for(;;)
  {
    // Wait for complete command (6 characters)
    if (uart_rx_index >= 7) {
      // Null terminate the command
      uart_cmd_buffer[7] = '\0';

      // Parse the command
      if (Command_Parse((char*)uart_cmd_buffer, &parsed_command)) {
        // Send parsed command to Control Task
        status = osMessageQueuePut(xCommandQueue, &parsed_command, 0, 100);
        if (status != osOK) {
          SendACK("QUEUE_FULL");
        }
      } else {
        SendACK("INVALID_CMD");
      }

      // Reset buffer
      uart_rx_index = 0;
      memset(uart_cmd_buffer, 0, sizeof(uart_cmd_buffer));
    }

    osDelay(10); // Small delay to prevent busy waiting
  }
  /* USER CODE END CommunicationTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  RobotCommand_t received_command;
  osStatus_t status;
  static bool timers_started = false;

  /* Infinite loop */
  for(;;)
  {
    if (!timers_started) {
      StartSensorTimers();
      timers_started = true;
    }

    if (s_ultrasonic_sample_pending) {
      s_ultrasonic_sample_pending = false;
      SharedData_UpdateUltrasonic(Ultrasonic_ReadDistance());
    }

    if (SharedData_CheckEmergencyStop()) {
      HandleEmergencyStop();
    }

    status = osMessageQueueGet(xCommandQueue, &received_command, NULL, 100);

    /* === RUN CURRENT MOTION GOAL === */
    if (g_motion_goal.active) {
        if (g_motion_goal.target_distance_cm > 0.0f) {
            Robot_MoveStraight(g_motion_goal.target_distance_cm, 3100, 3000);
        } else if (g_motion_goal.target_distance_cm < 0.0f) {
            Robot_MoveBackward(fabsf(g_motion_goal.target_distance_cm), 3100, 3000);
        } else if (fabsf(g_motion_goal.target_angle_deg) > 0.0f) {
            Robot_TurnUpdate();
        }
    } else if (g_motion_goal.completed && !g_motion_goal.ack_sent) {
        g_motion_goal.ack_sent = 1;
        g_motion_goal.completed = 0;
        command_in_progress = false;
        SendACK("CMD_COMPLETE");
    }

    if (status == osOK) {
      ProcessRobotCommand(&received_command);
    }

    osDelay(20); // ~50 Hz control loop
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN MotorTask */
  MotorCommand_t motor_cmd;
  osStatus_t status;

  /* Infinite loop */
  for(;;)
  {
    // Wait for motor commands from Control Task
    status = osMessageQueueGet(xMotorQueue, &motor_cmd, NULL, 100);
    if (status == osOK) {
      // Execute motor command
      switch (motor_cmd.cmd_type) {
        case MOTOR_CMD_STOP:
          Motors_Stop();
          break;

        case MOTOR_CMD_FORWARD:
          Motors_ForwardCounts(motor_cmd.left_pwm, motor_cmd.right_pwm);
          if (motor_cmd.duration_ms > 0) {
            osDelay(motor_cmd.duration_ms);
            Motors_Brake();
            Motors_Stop();
          }
          break;

        case MOTOR_CMD_BACKWARD:
          Motors_ReverseCounts(motor_cmd.left_pwm, motor_cmd.right_pwm);
          if(motor_cmd.duration_ms > 0){
        	  osDelay(motor_cmd.duration_ms);
        	  Motors_Brake();
        	  Motors_Stop();
          }
          break;

        case MOTOR_CMD_BRAKE:
          Motors_Brake();
          break;

        default:
          Motors_Stop();
          break;
      }

      // Handle servo if specified
      if (motor_cmd.servo_angle > 0) {
        // Set servo position (assuming servo control function exists)
        // Servo_SetPosition(motor_cmd.servo_angle);
      }
    }

    osDelay(10); // Motor control loop delay
  }
  /* USER CODE END MotorTask */
}

void IMUTask(void *argument)
{
  (void)argument;

  for(;;)
  {
    if (osSemaphoreAcquire(xIMUSem, osWaitForever) == osOK) {
      IMU_Update(&hi2c2, 0, GYRO_FULL_SCALE_250DPS);
      SharedData_UpdateIMU(IMU_GetYawLatest(), IMU_GetGyroZLatest());
    }
  }
}

void EncoderTask(void *argument)
{
	 (void)argument;

	  for (;;)
	  {
	    // wait for the periodic timer “tick”
	    osSemaphoreAcquire(xEncoderSem, osWaitForever);

	    // snapshot both counters *each time*
	    uint32_t left  = __HAL_TIM_GET_COUNTER(&encA.htim);
	    uint32_t right = __HAL_TIM_GET_COUNTER(&encC.htim);

	    SharedData_UpdateEncoders((int32_t)left, (int32_t)right);
	    // no delay here; the software timer period sets the rate
	  }
}

/* USER CODE BEGIN Header_OLEDTask */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLEDTask */
void OLEDTask(void *argument)
{
  /* USER CODE BEGIN OLEDTask */
  SharedSensorData_t sensor_data;
  char display_buffer[32];
  snprintf(display_buffer, sizeof(display_buffer), "TESTTTTTT");
  OLED_ShowString(0, 0, (uint8_t*)display_buffer);
  /* Infinite loop */
  for(;;)
  {
    // Get current sensor data
    if (SharedData_GetSensorData(&sensor_data)) {
      OLED_Clear();

      // Display detection data from RPi (if available)
      snprintf(display_buffer, sizeof(display_buffer), "Det: %s", sensor_data.detection_code);
      OLED_ShowString(0, 0, (uint8_t*)display_buffer);

      // Display robot state
      switch (current_robot_state) {
        case ROBOT_STATE_IDLE:
          OLED_ShowString(0, 16, (uint8_t*)"State: IDLE");
          break;
        case ROBOT_STATE_MOVING_FORWARD:
          OLED_ShowString(0, 16, (uint8_t*)"State: FORWARD");
          break;
        case ROBOT_STATE_MOVING_BACKWARD:
          OLED_ShowString(0, 16, (uint8_t*)"State: BACKWARD");
          break;
        case ROBOT_STATE_TURNING_LEFT:
          OLED_ShowString(0, 16, (uint8_t*)"State: TURN_L");
          break;
        case ROBOT_STATE_TURNING_RIGHT:
          OLED_ShowString(0, 16, (uint8_t*)"State: TURN_R");
          break;
        case ROBOT_STATE_EMERGENCY_STOP:
          OLED_ShowString(0, 16, (uint8_t*)"State: E_STOP");
          break;
        default:
          OLED_ShowString(0, 16, (uint8_t*)"State: ERROR");
          break;
      }

      // Display sensor readings
      snprintf(display_buffer, sizeof(display_buffer), "Dist: %.1fcm", sensor_data.front_distance_cm);
      //snprintf(display_buffer, sizeof(display_buffer), "Dist: %.1fcm", sensor_data.distance_traveled_cm);
      OLED_ShowString(0, 32, (uint8_t*)display_buffer);

      snprintf(display_buffer, sizeof(display_buffer), "Yaw: %.1fdeg", sensor_data.yaw_angle_deg);
      OLED_ShowString(0, 48, (uint8_t*)display_buffer);

      OLED_Refresh_Gram();
    }

    osDelay(OLED_UPDATE_PERIOD_MS);
  }
  /* USER CODE END OLEDTask */
}

/* USER CODE BEGIN Application */
/**
 * @brief Software timer callback for ultrasonic sensor
 */
void UltrasonicTimerCallback(void *argument) {
  (void)argument;
  s_ultrasonic_sample_pending = true;
}

/**
 * @brief Software timer callback for encoder reading
 */
//void EncoderTimerCallback(void *argument) {
//  (void)argument;
//
//  //update_cnt1(&encA);
//  //update_cnt1(&encC);
//
//  SharedData_UpdateEncoders((int32_t)encA.cnt1, (int32_t)encC.cnt1);
//}
void EncoderTimerCallback(void *argument) {
  (void)argument;

  if (xEncoderSem != NULL) {
    osSemaphoreRelease(xEncoderSem);
  }
}

/**
 * @brief Software timer callback for IMU reading
 */
void IMUTimerCallback(void *argument) {
  (void)argument;

  if (xIMUSem != NULL) {
    osSemaphoreRelease(xIMUSem);
  }
}

/**
 * @brief Software timer callback for IR sensor reading (stub)
 */
//void IRTimerCallback(void *argument) {
//  float left_distance, right_distance;
//
//  (void)argument; // Suppress unused parameter warning
//
//  // Read IR sensors (stub implementation)
//  left_distance = IR_ReadLeft();
//  right_distance = IR_ReadRight();
//
//  SharedData_UpdateIR(left_distance, right_distance);
//}

/**
 * @brief Process robot command from RPi
 */
void ProcessRobotCommand(RobotCommand_t* cmd) {
  MotorCommand_t motor_cmd;
  
  if (!cmd) return;
  
  SharedData_UpdateDetection(cmd->detection_data);
  memset(&motor_cmd, 0, sizeof(motor_cmd));
  
  // Process command based on type
  switch (cmd->cmd_type) {
    case CMD_FORWARD:
      current_robot_state = ROBOT_STATE_MOVING_FORWARD;
      g_motion_goal.target_distance_cm = (float)cmd->units;
      g_motion_goal.target_angle_deg = 0.0f;
      g_motion_goal.active = 1;
      g_motion_goal.started = 0;
      g_motion_goal.completed = 0;
      g_motion_goal.ack_sent = 0;
      g_motion_goal.start_distance_cm = 0.0f;
      g_motion_goal.start_yaw_deg = 0.0f;
      g_motion_goal.start_left_ticks = 0;
      g_motion_goal.start_right_ticks = 0;
      command_in_progress = true;
      break;
      
    case CMD_BACKWARD:
      current_robot_state = ROBOT_STATE_MOVING_BACKWARD;
      g_motion_goal.target_distance_cm = -(float)cmd->units;  // NEGATIVE = backward
      g_motion_goal.target_angle_deg   = 0.0f;
      g_motion_goal.active = 1;
      g_motion_goal.started = 0;
      g_motion_goal.completed = 0;
      g_motion_goal.ack_sent = 0;
      g_motion_goal.start_distance_cm = 0.0f;
      g_motion_goal.start_yaw_deg = 0.0f;
      g_motion_goal.start_left_ticks = 0;
      g_motion_goal.start_right_ticks = 0;
      g_motion_goal.turn_backward = 0;     // not a turn
      command_in_progress = true;
      break;

      
    case CMD_STOP:
      current_robot_state = ROBOT_STATE_IDLE;
      motor_cmd.cmd_type = MOTOR_CMD_STOP;
      osMessageQueuePut(xMotorQueue, &motor_cmd, 0, 100);
      g_motion_goal.active = 0;
      g_motion_goal.started = 0;
      g_motion_goal.completed = 0;
      g_motion_goal.ack_sent = 0;
      command_in_progress = false;
      SendACK("CMD_COMPLETE");
      break;
      
    case CMD_FORWARD_INDEFINITE:
      current_robot_state = ROBOT_STATE_MOVING_FORWARD;
      motor_cmd.cmd_type = MOTOR_CMD_FORWARD;
      motor_cmd.left_pwm = 3750;
      motor_cmd.right_pwm = 3800;
      motor_cmd.duration_ms = 0; // Indefinite
      osMessageQueuePut(xMotorQueue, &motor_cmd, 0, 100);
      g_motion_goal.active = 0;
      g_motion_goal.started = 0;
      g_motion_goal.completed = 0;
      g_motion_goal.ack_sent = 0;
      command_in_progress = false;
      SendACK("CMD_COMPLETE");
      break;
      
    case CMD_BACKWARD_INDEFINITE:
      current_robot_state = ROBOT_STATE_MOVING_BACKWARD;
      motor_cmd.cmd_type = MOTOR_CMD_BACKWARD;
      motor_cmd.left_pwm = 3750;
      motor_cmd.right_pwm = 3800;
      motor_cmd.duration_ms = 0; // Indefinite
      osMessageQueuePut(xMotorQueue, &motor_cmd, 0, 100);
      g_motion_goal.active = 0;
      g_motion_goal.started = 0;
      g_motion_goal.completed = 0;
      g_motion_goal.ack_sent = 0;
      command_in_progress = false;
      SendACK("CMD_COMPLETE");
      break;
      
    case CMD_TURN_LEFT_90:
    	current_robot_state = ROBOT_STATE_TURNING_LEFT;
		g_motion_goal.target_distance_cm = 0.0f;
		g_motion_goal.target_angle_deg   = -90.0f;  // negative = left
	    g_motion_goal.active             = 1;
		g_motion_goal.started            = 0;
		g_motion_goal.completed          = 0;
		g_motion_goal.ack_sent           = 0;
		command_in_progress              = true;
		  break;
      break;
      
    case CMD_TURN_RIGHT_90:
    	current_robot_state = ROBOT_STATE_TURNING_RIGHT;
		g_motion_goal.target_distance_cm = 0.0f;
		g_motion_goal.target_angle_deg   = 90.0f;   // positive = right
		g_motion_goal.active             = 1;
		g_motion_goal.started            = 0;
		g_motion_goal.completed          = 0;
		g_motion_goal.ack_sent           = 0;
		command_in_progress              = true;
		break;

    case CMD_TURN_LEFT_90_BACK:
      current_robot_state = ROBOT_STATE_TURNING_LEFT;
      g_motion_goal.target_distance_cm = 0.0f;
      g_motion_goal.target_angle_deg   = -90.0f;   // left
      g_motion_goal.turn_backward      = 1;        // <<< backward pivot
      g_motion_goal.active = 1; g_motion_goal.started = 0;
      g_motion_goal.completed = 0; g_motion_goal.ack_sent = 0;
      command_in_progress = true;
      break;

    case CMD_TURN_RIGHT_90_BACK:
      current_robot_state = ROBOT_STATE_TURNING_RIGHT;
      g_motion_goal.target_distance_cm = 0.0f;
      g_motion_goal.target_angle_deg   = 90.0f;    // right
      g_motion_goal.turn_backward      = 1;        // <<< backward pivot
      g_motion_goal.active = 1; g_motion_goal.started = 0;
      g_motion_goal.completed = 0; g_motion_goal.ack_sent = 0;
      command_in_progress = true;
      break;
      
    default:
      // Invalid command
      SendACK("INVALID_CMD");
      break;
  }
}

/**
 * @brief Send ACK message back to RPi
 */
void SendACK(const char* message) {
  char ack_buffer[32];
  
  snprintf(ack_buffer, sizeof(ack_buffer), "ACK:%s\r\n", message);
  HAL_UART_Transmit(&huart3, (uint8_t*)ack_buffer, strlen(ack_buffer), 100);
}

/**
 * @brief Handle emergency stop condition
 */
void HandleEmergencyStop(void) {
  MotorCommand_t emergency_cmd;
  
  current_robot_state = ROBOT_STATE_EMERGENCY_STOP;
  
  // Stop all motors immediately
  emergency_cmd.cmd_type = MOTOR_CMD_BRAKE;
  osMessageQueuePut(xMotorQueue, &emergency_cmd, 0, 0);
  
  // Send emergency notification
  SendACK("EMERGENCY_STOP");
  
  // Wait for emergency condition to clear
  //osDelay(1000);
  
  // Clear emergency stop
  SharedData_SetEmergencyStop(false);
  current_robot_state = ROBOT_STATE_IDLE;
  SendACK("EMERGENCY_CLEARED");
}

/**
 * @brief UART receive complete callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    if (uart_rx_index < 7) {
      uart_cmd_buffer[uart_rx_index] = uart_rx_buffer[0];
      uart_rx_index++;
    }
    
    // Continue receiving
    HAL_UART_Receive_IT(&huart3, &uart_rx_buffer[0], 1);
  }
}

/**
 * @brief Initialize and start all software timers
 */


void StartSensorTimers(void) {
  // Start all sensor timers
  osTimerStart(xUltrasonicTimer, ULTRASONIC_TIMER_PERIOD_MS);
  osTimerStart(xEncoderTimer, ENCODER_TIMER_PERIOD_MS);
  osTimerStart(xIMUTimer, IMU_TIMER_PERIOD_MS);
  // osTimerStart(xIRTimer, IR_TIMER_PERIOD_MS);
  if (xEncoderSem != NULL) {
    osSemaphoreRelease(xEncoderSem);
  }
  if (xIMUSem != NULL) {
    osSemaphoreRelease(xIMUSem);
  }
}
/* USER CODE END Application */
