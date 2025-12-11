/**
 * @file pid.h
 * @brief PID Controller interface
 * @author [Your Name/Author]
 * @date 2025-09-15
 */
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include <math.h>

/**
 * @brief Initialize PID gains and output limits
 * @param p Proportional gain
 * @param i Integral gain
 * @param d Derivative gain
 * @param min Minimum output saturation
 * @param max Maximum output saturation
 */
void pid_init(float p, float i, float d, float min, float max);

/**
 * @brief Reset the integrator accumulation to zero
 */
void pid_reset_integrator(void);

/**
 * @brief Compute PID output
 * @param target Desired setpoint
 * @param actual Current measured value
 * @param rate   Current rate of change (for D-term)
 * @param dt     Time delta in seconds
 * @return float Control output
 */
float pid_compute(float target, float actual, float rate, float dt);

#endif