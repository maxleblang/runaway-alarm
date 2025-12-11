/*
 * pwm.h
 *
 *  Created on: Jan 21, 2022
 *      Author: Joe Krachey
 */

#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"
#include "pins.h"


/* Public API */

/** Initialize the PWM bus to the specified pin
 *
 * @param pwm_pin - The GPIO pin to use for PWM
 * @param frequency_hz - The PWM frequency in Hz
 * @param initial_duty_cycle - Initial duty cycle as a percentage (0-100)
 * @param pwm_obj - Pointer to PWM object to initialize
 * @return The status of the initialization
 */
cy_rslt_t pwm_init(cyhal_gpio_t pwm_pin,
                   uint32_t frequency_hz,
                   float initial_duty_cycle,
                   cyhal_pwm_t *pwm_obj);

#endif /* I2C_H_ */
