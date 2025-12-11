/**
 * @file pid.c
 * @brief PID Controller implementation
 */
#include "pid.h"
#include <math.h>

// PID Gains
static float Kp = 0.0f;
static float Ki = 0.0f;
static float Kd = 0.0f;

// State and Limits
static float integrator = 0.0f;
static float lim_min = -1.0f;
static float lim_max = 1.0f;

/**
 * @brief Initialize PID gains and output limits
 */
void pid_init(float p, float i, float d, float min, float max)
{
    Kp = p;
    Ki = i;
    Kd = d;
    lim_min = min;
    lim_max = max;
    integrator = 0.0f;
}

/**
 * @brief Reset the integrator to zero
 */
void pid_reset_integrator(void)
{
    integrator = 0.0f;
}

/**
 * @brief Compute PID output
 * @param target Desired angle (0.0 for balancing)
 * @param actual Current estimated angle (from Kalman)
 * @param rate   Current angular rate (from Gyro)
 * @param dt     Time delta in seconds
 */
float pid_compute(float target, float actual, float rate, float dt)
{
    float error = target - actual;
    
    // Deadband: if error is very small, treat it as zero to prevent rumbling
    if (fabsf(error) < 1.0f)
    {
        error = 0.0f;
    }
    
    // P Term
    float P_term = Kp * error;

    // I Term
    integrator += error * dt;
    float I_term = Ki * integrator;

    // D Term (opposes angular velocity to dampen oscillation)
    float D_term = Kd * rate;

    // Summation
    float output = P_term + I_term + D_term;

    // Output deadband: ignore small outputs that won't overcome static friction
    if (fabsf(output) < 8.0f)
    {
        output = 0.0f;
    }

    // Saturation and Anti-Windup
    if (output > lim_max)
    {
        output = lim_max;
        integrator -= error * dt; // Prevent integrator buildup
    }
    else if (output < lim_min)
    {
        output = lim_min;
        integrator -= error * dt; // Prevent integrator buildup
    }

    return output;
}