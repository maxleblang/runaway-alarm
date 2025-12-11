#ifndef __TASK_MOTOR_H__
#define __TASK_MOTOR_H__

#include "main.h"
#include "dc_motor.h"
#include "FreeRTOS.h"
#include "queue.h"


/* Motor message structure */
typedef struct
{
    dc_motor_direction_t direction;
    uint8_t speed;
} motor_message_t;

extern QueueHandle_t q_motor;
void task_motor_init(void);
void task_motor_get_pwm_objects(cyhal_pwm_t **pwm1, cyhal_pwm_t **pwm2, cyhal_pwm_t **pwm3, cyhal_pwm_t **pwm4);

#endif
