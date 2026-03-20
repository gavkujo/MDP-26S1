///*
// * ir_sensor.c
// *
// *  Created on: Sep 25, 2025
// *      Author: MDP Team
// */
//
//#include "ir_sensor.h"
//
//#include "ir_sensor.h"
//#include <math.h>
//#include <stdlib.h> // for qsort
//
//// Internal: IR sensor models for left and right
//static int IR_LeftModel  = IR_MODEL_GP2Y0A21Y;
//static int IR_RightModel = IR_MODEL_GP2Y0A21Y;
//
//// Stub: replace with your STM ADC read function
//static uint16_t ADC_Read(uint8_t channel) {
//    ADC_ChannelConfTypeDef sConfig = {0};
//    // change the above line to your ADC handle
//    // e.g., ADC_HandleTypeDef hadc1; and initialize it properly in main.c or prob done by cubemx
//    sConfig.Channel = channel;
//    sConfig.Rank = 1;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; // adjust as needed
//
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, 100);
//    return HAL_ADC_GetValue(&hadc1);
//}
//
//void IR_Init(void) {
//    HAL_ADC_Start(&hadc1); // Start ADC (adjust hadc1 to your ADC handle)
//}
//
//// Helper: sort array for median calculation
//static void sort_samples(uint16_t *array, size_t size) {
//    // Simple bubble sort
//    for(size_t i = 0; i < size-1; i++) {
//        bool swapped = false;
//        for(size_t j = 0; j < size-i-1; j++) {
//            if(array[j] > array[j+1]) {
//                uint16_t tmp = array[j];
//                array[j] = array[j+1];
//                array[j+1] = tmp;
//                swapped = true;
//            }
//        }
//        if(!swapped) break;
//    }
//}
//
//// Internal: convert ADC value to distance in cm
//static float ADCToDistance(uint16_t adc_val, int model) {
//    float voltage;   // in mV
//    float distance;  // in cm
//
//    // For STM32 assume 12-bit ADC (0-4095) and 3.3V reference
//    voltage = (adc_val / 4095.0f) * 3300.0f;
//
//    switch(model) {
//        case IR_MODEL_GP2Y0A21Y:
//            distance = 27.728f * powf(voltage / 1000.0f, -1.2045f);
//            break;
//        case IR_MODEL_GP2Y0A02YK:
//            distance = 60.374f * powf(voltage / 1000.0f, -1.16f);
//            break;
//        case IR_MODEL_GP2Y0A430:
//            distance = 12.08f * powf(voltage / 1000.0f, -1.058f);
//            break;
//        case IR_MODEL_GP2Y0A710K0F:
//            if(voltage < 1400.0f || voltage > 3300.0f) {
//                distance = 0.0f; // invalid reading
//            } else {
//                distance = 1.0f / (((voltage - 1125.0f)/1000.0f) / 137.5f);
//            }
//            break;
//        default:
//            distance = 0.0f; // unknown model
//            break;
//    }
//
//    return distance;
//}
//
//// Read sensor median
//static float IR_ReadSensor(uint8_t channel, int model) {
//    uint16_t samples[IR_NB_SAMPLE];
//
//    for(size_t i = 0; i < IR_NB_SAMPLE; i++) {
//        samples[i] = ADC_Read(channel);
//    }
//
//    sort_samples(samples, IR_NB_SAMPLE);
//
//    // median
//    uint16_t median_val = samples[IR_NB_SAMPLE/2];
//
//    return ADCToDistance(median_val, model);
//}
//
//// Public APIs
//
////PLEASE INIT THE PINS
//// Configure ADC1 (or whichever ADC you want) in CubeMX:
//// Set scan mode off, continuous conversion off.
//// Configure PA0 (OR WHATEVER) as ADC_IN0 (left IR) and PA1 (OR WHATEVER) as ADC_IN1 (right IR).
//
//float IR_ReadLeft(void) {
//    return IR_ReadSensor(IR_LEFT_ADC_CHANNEL, IR_LeftModel);
//}
//
//float IR_ReadRight(void) {
//    return IR_ReadSensor(IR_RIGHT_ADC_CHANNEL, IR_RightModel);
//}
//
//bool IR_IsLeftWallDetected(float threshold_cm) {
//    return IR_ReadLeft() < threshold_cm;
//}
//
//bool IR_IsRightWallDetected(float threshold_cm) {
//    return IR_ReadRight() < threshold_cm;
//}
//
//void IR_Calibrate(void) {
//    // TODO: implement calibration routine (if needed)
//}
//
