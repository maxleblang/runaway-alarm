/*
 * servo.h
 *
 *  Created on: Oct 20, 2020
 *      Author: Joe Krachey
 */

#ifndef __TASK_SERVO_H__
#define __TASK_SERVO_H__

#include "main.h"
#include "task_console.h"

#define PWM_SERVO_FREQ_HZ     50      /* 50Hz frequency for servo control */
#define PWM_INIT_DUTY_CYCLE   7.5f    /* Initial duty cycle (center position) */


void task_servo_init(void);


#endif
