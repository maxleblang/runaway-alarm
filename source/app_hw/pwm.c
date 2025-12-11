#include "pins.h"

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"


cy_rslt_t pwm_init(cyhal_gpio_t pwm_pin,
                   uint32_t frequency_hz,
                   float initial_duty_cycle,
                   cyhal_pwm_t *pwm_obj)
{
    cy_rslt_t result;

    /* Initialize PWM on the specified pin */
    result = cyhal_pwm_init(pwm_obj, pwm_pin, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Set PWM frequency and initial duty cycle */
    result = cyhal_pwm_set_duty_cycle(pwm_obj, initial_duty_cycle, frequency_hz);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Start the PWM output */
    result = cyhal_pwm_start(pwm_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    return CY_RSLT_SUCCESS;
}