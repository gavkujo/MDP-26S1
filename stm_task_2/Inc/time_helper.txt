/*
 * time_helper.h
 *
 *  Created on: Sep 12, 2025
 *      Author: GARV001
 */

// time_helper.h
#ifndef TIME_HELPER_H
#define TIME_HELPER_H

#include <stdint.h>

void    time_helper_init(void);   // call once at startup
uint32_t time_micros(void);       // current time in µs (wraps)
float   dt_seconds(void);         // delta seconds since last call

#endif // TIME_HELPER_H
