#include "dc_motor.h"
#include <stdint.h>
#include "cyhal_pwm.h"
#include "pins.h"

// Internal state variables
static dc_motor_direction_t current_direction = DC_MOTOR_STOP;
static uint8_t current_speed = 0;

cy_rslt_t dc_motor_init(void)
{
	// Initialize internal state
	current_direction = DC_MOTOR_STOP;
	current_speed = 0;
	
	// No GPIO initialization needed since we're using PWM
	return CY_RSLT_SUCCESS;
}

cy_rslt_t dc_motor_pwm_init(cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4)
{
	cy_rslt_t result;

	// Initialize PWM object for IN1
	result = cyhal_pwm_init(pwm_obj_1, MOTOR1_1, NULL);
	if (result != CY_RSLT_SUCCESS) {
		return result;
	}
	cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000); // Start with 0% duty cycle
	cyhal_pwm_start(pwm_obj_1);

	// Initialize PWM object for IN2
	result = cyhal_pwm_init(pwm_obj_2, MOTOR1_2, NULL);
	if (result != CY_RSLT_SUCCESS) {
		cyhal_pwm_free(pwm_obj_1);
		return result;
	}
	cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000); // Start with 0% duty cycle
	cyhal_pwm_start(pwm_obj_2);

	// Initialize PWM object for Motor 2 IN1
	result = cyhal_pwm_init(pwm_obj_3, MOTOR2_1, NULL);
	if (result != CY_RSLT_SUCCESS) {
		cyhal_pwm_free(pwm_obj_1);
		cyhal_pwm_free(pwm_obj_2);
		return result;
	}
	cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000); // Start with 0% duty cycle
	cyhal_pwm_start(pwm_obj_3);

	// Initialize PWM object for Motor 2 IN2
	result = cyhal_pwm_init(pwm_obj_4, MOTOR2_2, NULL);
	if (result != CY_RSLT_SUCCESS) {
		cyhal_pwm_free(pwm_obj_1);
		cyhal_pwm_free(pwm_obj_2);
		cyhal_pwm_free(pwm_obj_3);
		return result;
	}
	cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000); // Start with 0% duty cycle
	cyhal_pwm_start(pwm_obj_4);

	return CY_RSLT_SUCCESS;
}

void dc_motor_set_direction(dc_motor_direction_t direction, cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4)
{
	// Set IN1/IN2 pins based on direction using H-Bridge truth table from DRV8251 datasheet
	switch (direction) {	
		case DC_MOTOR_REVERSE:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// IN1=0, IN2=1: Reverse
			cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, current_speed, 1000); // Convert percentage to duty cycle

			cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, current_speed, 1000);

			//start the pwm signals again
			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);
			break;
			
		case DC_MOTOR_FORWARD:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// IN1=1, IN2=0: Forward Motor 1
			cyhal_pwm_set_duty_cycle(pwm_obj_1, current_speed, 1000); // Convert percentage to duty cycle
			cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);

			// IN1=1, IN2=0: Forward Motor 2
			cyhal_pwm_set_duty_cycle(pwm_obj_3, current_speed, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);

			//start the pwm signals again
			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);
			break;

		case DC_MOTOR_LEFT:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// Differential steer: left motor reverse, right motor forward
			// Left motor (Motor1): IN1=0, IN2=PWM
			cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, current_speed, 1000);
			// Right motor (Motor2): IN1=PWM, IN2=0
			cyhal_pwm_set_duty_cycle(pwm_obj_3, current_speed, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);

			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);
			break;

		case DC_MOTOR_RIGHT:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// Differential steer: left motor forward, right motor reverse
			// Left motor (Motor1): IN1=PWM, IN2=0
			cyhal_pwm_set_duty_cycle(pwm_obj_1, current_speed, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
			// Right motor (Motor2): IN1=0, IN2=PWM
			cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, current_speed, 1000);

			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);
			break;
	
			
		case DC_MOTOR_STOP:
		default:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// Default to coast for safety
			cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);
			
			//start the pwm signals again
			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);
			break;
	}
	current_direction = direction;
}

void dc_motor_set_speed(uint8_t percent, dc_motor_direction_t direction, cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4)
{
	// Clamp speed to 0-100%
	if (percent > 100) {
		percent = 100;
	}
	
	current_speed = percent;
	
	// Apply speed based on current direction
	switch (direction) {
		case DC_MOTOR_FORWARD:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// IN1=PWM, IN2=0
			cyhal_pwm_set_duty_cycle(pwm_obj_1, percent, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_3, percent, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);
			
			//start the pwm signals again
			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);

			break;
			
		case DC_MOTOR_REVERSE:
		//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);

			// IN1=0, IN2=PWM
			cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, percent, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, percent, 1000);
			
			
			//start the pwm signals again
			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);

			break;

			case DC_MOTOR_LEFT:
				//Stop the pwm signals
				cyhal_pwm_stop(pwm_obj_1);
				cyhal_pwm_stop(pwm_obj_2);
				cyhal_pwm_stop(pwm_obj_3);
				cyhal_pwm_stop(pwm_obj_4);
				// Left motor reverse, right motor forward
				cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
				cyhal_pwm_set_duty_cycle(pwm_obj_2, percent, 1000);
				cyhal_pwm_set_duty_cycle(pwm_obj_3, percent, 1000);
				cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);
				cyhal_pwm_start(pwm_obj_1);
				cyhal_pwm_start(pwm_obj_2);
				cyhal_pwm_start(pwm_obj_3);
				cyhal_pwm_start(pwm_obj_4);
				break;

			case DC_MOTOR_RIGHT:
				//Stop the pwm signals
				cyhal_pwm_stop(pwm_obj_1);
				cyhal_pwm_stop(pwm_obj_2);
				cyhal_pwm_stop(pwm_obj_3);
				cyhal_pwm_stop(pwm_obj_4);
				// Left motor forward, right motor reverse
				cyhal_pwm_set_duty_cycle(pwm_obj_1, percent, 1000);
				cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
				cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
				cyhal_pwm_set_duty_cycle(pwm_obj_4, percent, 1000);
				cyhal_pwm_start(pwm_obj_1);
				cyhal_pwm_start(pwm_obj_2);
				cyhal_pwm_start(pwm_obj_3);
				cyhal_pwm_start(pwm_obj_4);
				break;
			
		case DC_MOTOR_STOP:
		default:
			//Stop the pwm signals
			cyhal_pwm_stop(pwm_obj_1);
			cyhal_pwm_stop(pwm_obj_2);
			cyhal_pwm_stop(pwm_obj_3);
			cyhal_pwm_stop(pwm_obj_4);
			// Both pins low for coast/stop
			cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
			cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);
			//start the pwm signals again
			cyhal_pwm_start(pwm_obj_1);
			cyhal_pwm_start(pwm_obj_2);
			cyhal_pwm_start(pwm_obj_3);
			cyhal_pwm_start(pwm_obj_4);
			break;
	}
	
	current_direction = direction;
}

void dc_motor_set_independent(int8_t left_speed, int8_t right_speed, cyhal_pwm_t *pwm_obj_1, cyhal_pwm_t *pwm_obj_2, cyhal_pwm_t *pwm_obj_3, cyhal_pwm_t *pwm_obj_4)
{
	// Stop all PWM signals
	cyhal_pwm_stop(pwm_obj_1);
	cyhal_pwm_stop(pwm_obj_2);
	cyhal_pwm_stop(pwm_obj_3);
	cyhal_pwm_stop(pwm_obj_4);
	
	// Left motor (Motor1) - pwm_obj_1 = IN1, pwm_obj_2 = IN2
	uint8_t left_abs = (left_speed < 0) ? -left_speed : left_speed;
	if (left_abs > 100) left_abs = 100;
	
	if (left_speed > 0) {
		// Forward: IN1=PWM, IN2=0
		cyhal_pwm_set_duty_cycle(pwm_obj_1, left_abs, 1000);
		cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
	} else if (left_speed < 0) {
		// Reverse: IN1=0, IN2=PWM
		cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
		cyhal_pwm_set_duty_cycle(pwm_obj_2, left_abs, 1000);
	} else {
		// Stop
		cyhal_pwm_set_duty_cycle(pwm_obj_1, 0, 1000);
		cyhal_pwm_set_duty_cycle(pwm_obj_2, 0, 1000);
	}
	
	// Right motor (Motor2) - pwm_obj_3 = IN1, pwm_obj_4 = IN2
	uint8_t right_abs = (right_speed < 0) ? -right_speed : right_speed;
	if (right_abs > 100) right_abs = 100;
	
	if (right_speed > 0) {
		// Forward: IN1=PWM, IN2=0
		cyhal_pwm_set_duty_cycle(pwm_obj_3, right_abs, 1000);
		cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);
	} else if (right_speed < 0) {
		// Reverse: IN1=0, IN2=PWM
		cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
		cyhal_pwm_set_duty_cycle(pwm_obj_4, right_abs, 1000);
	} else {
		// Stop
		cyhal_pwm_set_duty_cycle(pwm_obj_3, 0, 1000);
		cyhal_pwm_set_duty_cycle(pwm_obj_4, 0, 1000);
	}
	
	// Start all PWM signals
	cyhal_pwm_start(pwm_obj_1);
	cyhal_pwm_start(pwm_obj_2);
	cyhal_pwm_start(pwm_obj_3);
	cyhal_pwm_start(pwm_obj_4);
}

dc_motor_direction_t dc_motor_get_direction(void)
{
	return current_direction;
}
