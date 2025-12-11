#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "cyhal.h"
#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>
#include "dc_motor.h"   // for dc_motor_direction_t
#include "cyhal_pwm.h"

/**
 * Start interrupt-driven edge counting on the 4 encoder pins:
 *   MOTOR1_ENC1, MOTOR1_ENC2, MOTOR2_ENC1, MOTOR2_ENC2
 * Spawns an internal FreeRTOS control task that runs every 500 ms
 * to balance the two motors by adjusting their PWM duty cycles.
 */
void encoder_start_edge_counting(void);

/**
 * Register the PWM handles for the two H-bridge inputs per motor.
 * Order must match your existing wiring:
 *   pwm_m1_in1 = MOTOR1_1, pwm_m1_in2 = MOTOR1_2
 *   pwm_m2_in1 = MOTOR2_1, pwm_m2_in2 = MOTOR2_2
 */
void encoder_register_pwm_objects(cyhal_pwm_t *pwm_m1_in1,
                                  cyhal_pwm_t *pwm_m1_in2,
                                  cyhal_pwm_t *pwm_m2_in1,
                                  cyhal_pwm_t *pwm_m2_in2);

/**
 * Update the target/base speed (0–100 %) the controller should
 * maintain for both motors before applying balancing corrections.
 */
void encoder_set_base_speed(uint8_t percent);

/**
 * Update the current direction so the controller knows which pins
 * carry PWM when it nudges each motor independently.
 */
void encoder_set_base_direction(dc_motor_direction_t dir);

/* Optional: fetch the most recent 0.5 s counts (for debugging) */
void encoder_get_last_counts(int32_t *m1_count, int32_t *m2_count);

#endif /* __ENCODER_H__ */
