#ifndef DC_MOTOR_H
#define DC_MOTOR_H

// #include "main.h"
#include "pins.h"
#include "cyhal_pwm.h"

typedef enum {
	DC_MOTOR_STOP = 0,
	DC_MOTOR_FORWARD,
	DC_MOTOR_REVERSE,
	DC_MOTOR_LEFT,     // differential turn: left motor reverse, right motor forward
	DC_MOTOR_RIGHT,    // differential turn: left motor forward, right motor reverse
} dc_motor_direction_t;

// Initialize motor driver (GPIOs)
cy_rslt_t dc_motor_init(void);

// Initialize motor driver PWM objects
cy_rslt_t dc_motor_pwm_init(cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4);

// Set motor direction (forward, reverse, brake, coast)
void dc_motor_set_direction(dc_motor_direction_t direction, cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4);

// Set motor speed (0-100% duty cycle)
void dc_motor_set_speed(uint8_t percent, dc_motor_direction_t direction, cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4);

// Set left and right motors independently (speed: -100 to +100, negative = reverse)
void dc_motor_set_independent(int8_t left_speed, int8_t right_speed, cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4);

// (Optional) Get current motor state
dc_motor_direction_t dc_motor_get_direction(void);

#endif // DC_MOTOR_H
