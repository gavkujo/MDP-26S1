///*
// * ir_sensor.h
// *
// *  Created on: Sep 25, 2025
// *      Author: MDP Team
// */
//
//#ifndef INC_IR_SENSOR_H_
//#define INC_IR_SENSOR_H_
//
//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>
//
//// ADC channels for IR sensors
//#define IR_LEFT_ADC_CHANNEL    ADC_CHANNEL_0   // PA0 or whatever pin
//#define IR_RIGHT_ADC_CHANNEL   ADC_CHANNEL_1   // PA1 or whatever pin
//
//// IR sensor models
//#define IR_MODEL_GP2Y0A21Y     1080    // 10–80 cm
//#define IR_MODEL_GP2Y0A02YK    20150   // 20–150 cm
//#define IR_MODEL_GP2Y0A710K0F 100500  // 100–500 cm
//#define IR_MODEL_GP2Y0A430     430     // 4–30 cm (example)
//
//// Number of ADC samples for median calculation
//#define IR_NB_SAMPLE           25
//
//// Function Prototypes
//void IR_Init(void);
//
//float IR_ReadLeft(void);
//float IR_ReadRight(void);
//
//bool IR_IsLeftWallDetected(float threshold_cm);
//bool IR_IsRightWallDetected(float threshold_cm);
//
//void IR_Calibrate(void);
//
//#endif /* INC_IR_SENSOR_H_ */
