/*
 * task_pid.c
 *
 *  Created on: Nov 12, 2025
 */

#include "task_pid.h"
#include "cy_syslib.h"
#include "dc_motor.h"
#include "task_console.h"
#include "task_motor.h"
#include "task_imu.h"
#include "imu.h"
#include <math.h>
#include <stdint.h>

#define PID_KP_VALUE 13.5f    // Proportional gain (more aggressive for static friction)
#define PID_KI_VALUE 0.25f    // Integral gain (helps overcome static friction)
#define PID_KD_VALUE 2.1f     // Derivative gain (moderate damping)
#define PID_OUTPUT_MIN -100.0f // Min output
#define PID_OUTPUT_MAX 100.0f  // Max output
#define PID_DESIRED_PITCH BALANCE_PITCH // Desired pitch angle in degrees
#define PID_DT (1.0f / 208.0f) // Time delta for 208 Hz sampling

volatile bool pid_enabled = true;
volatile turn_state_t turn_state = TURN_OFF;

// Desired heading with mutex for thread-safe access
static float pid_desired_heading = BALANCE_PITCH;
static SemaphoreHandle_t pid_desired_heading_mutex = NULL;

static KalmanFilter kalman_filter;
static KalmanFilter kalman_filter_yaw;
static float current_yaw = 0.0f;
static float turn_start_yaw = 0.0f;
static bool turn_in_progress = false;
static pid_state_t pid_state = {0};

static BaseType_t cli_handler_pid_on(char *buf, size_t len, const char *cmd) {
  pid_enabled = true;
  task_print("pid on\n");
  return pdFALSE;
}

static BaseType_t cli_handler_pid_off(char *buf, size_t len, const char *cmd) {
  pid_enabled = false;
  task_print("pid off\n");
  return pdFALSE;
}

static BaseType_t cli_handler_turn_left(char *buf, size_t len, const char *cmd) {
  turn_state = TURN_LEFT;
  task_print("turning left\n");
  return pdFALSE;
}

static BaseType_t cli_handler_turn_right(char *buf, size_t len, const char *cmd) {
  turn_state = TURN_RIGHT;
  task_print("turning right\n");
  return pdFALSE;
}

static BaseType_t cli_handler_turn_off(char *buf, size_t len, const char *cmd) {
  turn_state = TURN_OFF;
  task_print("turn off\n");
  return pdFALSE;
}

static const CLI_Command_Definition_t cmd_pid_on = {
    "pid_on",           /* command text */
    "\r\npid on\r\n",   /* command help text */
    cli_handler_pid_on, /* handler function */
    0                   /* requires 0 parameters */
};

static const CLI_Command_Definition_t cmd_pid_off = {
    "pid_off",           /* command text */
    "\r\npid off\r\n",   /* command help text */
    cli_handler_pid_off, /* handler function */
    0                    /* requires 0 parameters */
};

static const CLI_Command_Definition_t cmd_turn_left = {
    "turn_left",           /* command text */
    "\r\nturn_left\r\n",   /* command help text */
    cli_handler_turn_left, /* handler function */
    0                      /* requires 0 parameters */
};

static const CLI_Command_Definition_t cmd_turn_right = {
    "turn_right",           /* command text */
    "\r\nturn_right\r\n",   /* command help text */
    cli_handler_turn_right, /* handler function */
    0                       /* requires 0 parameters */
};

static const CLI_Command_Definition_t cmd_turn_off = {
    "turn_off",           /* command text */
    "\r\nturn_off\r\n",   /* command help text */
    cli_handler_turn_off, /* handler function */
    0                     /* requires 0 parameters */
};

/**
 * @brief Set the desired pitch angle (thread-safe)
 * @param desired_pitch Desired pitch angle in degrees
 */
void task_pid_set_desired_pitch(float desired_pitch)
{
    if (pid_desired_heading_mutex != NULL)
    {
        if (xSemaphoreTake(pid_desired_heading_mutex, portMAX_DELAY) == pdTRUE)
        {
            pid_desired_heading = desired_pitch;
            xSemaphoreGive(pid_desired_heading_mutex);
        }
    }
}



/**
 * @brief Start a non-blocking 90 degree left turn
 * Call task_pid_is_turn_complete() to check when turn is finished
 */
void task_pid_start_turn_left(void)
{
    turn_start_yaw = current_yaw;
    turn_state = TURN_LEFT;
    turn_in_progress = true;
}

/**
 * @brief Check if the current turn is complete
 * @return true if turn has completed 90 degrees, false otherwise
 */
bool task_pid_is_turn_complete(void)
{
    if (!turn_in_progress)
    {
        return true; // No turn in progress means complete
    }
    
    // Check if we've turned 90 degrees
    if (fabsf(current_yaw - turn_start_yaw) >= FULL_TURN_THRESHOLD)
    {
        turn_state = TURN_OFF;
        turn_in_progress = false;
        return true;
    }
    
    return false;
}

/**
 * @brief Calculate PID output using Kalman-filtered IMU data
 * @param Pointer to PID struct
 * @param gyro_rt Raw gyroscope angular rate (dps from IMU)
 * @param ax Raw accelerometer X-axis (g from IMU)
 * @return float PID control output
 */
// Complementary filter state
static float comp_pitch = 0.0f;
#define COMP_ALPHA 0.90f  // Trust gyro 90%, accel 10% (faster angle detection)

float task_pid_calculate_output(pid_state_t *s, int16_t gyro_rt, int16_t ax)
{
    int16_t gyro_rate_offset = gyro_rt + PTCH_RT_OFFSET;
    int16_t accel_x_offset = ax + AX_OFFSET;
    
    // Convert accel to g, then to pitch angle
    float accel_g = (float)accel_x_offset / ACCEL_SCALE;
    // Clamp to valid asin range
    if (accel_g > 1.0f) accel_g = 1.0f;
    if (accel_g < -1.0f) accel_g = -1.0f;
    float accel_pitch_deg = asinf(accel_g) * 57.2958f;
    
    // Convert gyro to degrees per second
    float gyro_rate_dps = (float)gyro_rate_offset / GYRO_SCALE;

    // Complementary filter: fast gyro integration + slow accel correction
    comp_pitch = COMP_ALPHA * (comp_pitch + gyro_rate_dps * PID_DT) + (1.0f - COMP_ALPHA) * accel_pitch_deg;
    float filtered_pitch = comp_pitch;
    
    // Store state
    s->pitch = (int32_t)(filtered_pitch * 1000); 
    s->pitch_int += (int32_t)(filtered_pitch * 100);
    
    // Get desired pitch with mutex protection
    float desired_pitch_local = PID_DESIRED_PITCH; // Default fallback
    if (pid_desired_heading_mutex != NULL)
    {
        if (xSemaphoreTake(pid_desired_heading_mutex, 0) == pdTRUE)
        {
            desired_pitch_local = pid_desired_heading;
            xSemaphoreGive(pid_desired_heading_mutex);
        }
    }
    
    // Compute PID output
    float pid_output = pid_compute(
        desired_pitch_local,
        filtered_pitch,
        gyro_rate_dps,
        PID_DT
    );
    
    return pid_output;
}

/**
 * @brief FreeRTOS task that runs the PID control loop
 * Reads IMU data, calculates PID output, and commands motors
 */
static void task_pid(void *param)
{
    (void)param;
    
    imu_packet_t imu_request;
    imu_request.return_queue = q_imu_rsp;
    
    for (;;)
    {
        // Check if we should process (either PID enabled OR turning active)
        if (!pid_enabled && turn_state == TURN_OFF)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        
        // Request IMU data
        imu_request.address = 0; // 0 means read all accel/gyro
        imu_request.value = 0;
        
        if (xQueueSend(q_imu_req, &imu_request, portMAX_DELAY) != pdPASS)
        {
            task_print_error("Failed to send IMU request\r\n");
            continue;
        }
        
        // Wait for IMU response
        if (xQueueReceive(q_imu_rsp, &imu_request, portMAX_DELAY) != pdPASS)
        {
            task_print_error("Failed to receive IMU response\r\n");
            continue;
        }
        
        // Calculate PID output using gyro_y (pitch rate) and accel_x (pitch accel)
        float pid_output = task_pid_calculate_output(&pid_state, (int16_t)imu_request.gy, (int16_t)imu_request.ax);
        
        // Update yaw tracking using Kalman filter with gyro Z
        float gyro_z_dps = (float)imu_request.gz / GYRO_SCALE;
        current_yaw = imu_kalman_update(&kalman_filter_yaw, current_yaw + gyro_z_dps * PID_DT, gyro_z_dps, PID_DT);
        
        // Apply turn offset to create differential steering while maintaining balance
        float left_motor_output = pid_output;
        float right_motor_output = pid_output;
        
        if (turn_state == TURN_LEFT)
        {
            // Left motor gets less power (or reverses), right motor gets more
            left_motor_output -= TURN_OFFSET;
            right_motor_output += TURN_OFFSET;
        }
        else if (turn_state == TURN_RIGHT)
        {
            // Right motor gets less power (or reverses), left motor gets more
            left_motor_output += TURN_OFFSET;
            right_motor_output -= TURN_OFFSET;
        }
        
        // Send motor commands
        if (turn_state != TURN_OFF)
        {
            // Use independent motor control for differential steering
            static cyhal_pwm_t *pwm1 = NULL, *pwm2 = NULL, *pwm3 = NULL, *pwm4 = NULL;
            if (pwm1 == NULL) {
                task_motor_get_pwm_objects(&pwm1, &pwm2, &pwm3, &pwm4);
            }
            
            // Clamp outputs to -100 to +100 range
            int8_t left_speed = (int8_t)fmaxf(-100.0f, fminf(100.0f, left_motor_output));
            int8_t right_speed = (int8_t)fmaxf(-100.0f, fminf(100.0f, right_motor_output));
            
            dc_motor_set_independent(left_speed, right_speed, pwm1, pwm2, pwm3, pwm4);
        }
        else
        {
            // Normal balancing mode - use queue-based motor control
            motor_message_t motor_cmd;
            float abs_output = fabsf(pid_output);
            
            if (pid_output > 0)
            {
                motor_cmd.direction = DC_MOTOR_FORWARD;
                motor_cmd.speed = (uint8_t)fminf(100.0f, abs_output);
            }
            else if (pid_output < 0)
            {
                motor_cmd.direction = DC_MOTOR_REVERSE;
                motor_cmd.speed = (uint8_t)fminf(100.0f, abs_output);
            }
            else
            {
                motor_cmd.direction = DC_MOTOR_FORWARD;
                motor_cmd.speed = 0;
            }
            
            if (xQueueSend(q_motor, &motor_cmd, portMAX_DELAY) != pdPASS)
            {
                task_print_error("Failed to send motor command\r\n");
            }
        }
        
        // Run at 208 Hz (5 ms period)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void task_pid_init(void)
{
    // Initialize Kalman filters
    imu_kalman_init(&kalman_filter);
    imu_kalman_init(&kalman_filter_yaw);
    current_yaw = 0.0f;
    
    // Initialize PID state
    pid_state.pitch = 0;
    pid_state.pitch_int = 0;
    
    // Initialize desired heading and its mutex
    pid_desired_heading = 0.0f;
    pid_desired_heading_mutex = xSemaphoreCreateMutex();
    
    // Initialize PID controller with gains and output limits
    pid_init(PID_KP_VALUE, PID_KI_VALUE, PID_KD_VALUE, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    
    // Register CLI commands
    FreeRTOS_CLIRegisterCommand(&cmd_pid_on);
    FreeRTOS_CLIRegisterCommand(&cmd_pid_off);
    FreeRTOS_CLIRegisterCommand(&cmd_turn_left);
    FreeRTOS_CLIRegisterCommand(&cmd_turn_right);
    FreeRTOS_CLIRegisterCommand(&cmd_turn_off);
    
    // Create PID control task
    xTaskCreate(
        task_pid,
        "Task_PID",
        configMINIMAL_STACK_SIZE + 128,
        NULL,
        configMAX_PRIORITIES - 4,
        NULL);
}
