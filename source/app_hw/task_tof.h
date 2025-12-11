/**
 * @file task_tof.h
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief FreeRTOS task for TOF (Time-of-Flight) sensor management
 * @version 0.1
 * @date 2025-12-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __TASK_TOF_H__
#define __TASK_TOF_H__

#include "main.h"
#include "task_console.h"


#define OBJECT_AVOIDANCE_DISTANCE_MM 600 // Distance threshold for object avoidance in mm

/**
 * @brief Initialize the TOF task and register CLI commands
 */
void task_tof_init(void);
void task_tof_reset_object_state(void);

#endif
