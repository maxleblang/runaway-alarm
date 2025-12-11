#ifndef __TASK_PID_H__
#define __TASK_PID_H__

#include "main.h"
#include "pid.h"
#include "task_console.h"
#include "task_imu.h"
#include "task_motor.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define PTCH_RT_OFFSET -190
#define AX_OFFSET -1242
#define GYRO_SCALE 131.0f   // LSB per dps for 250dps range (32768/250)
#define ACCEL_SCALE 16384.0f // LSB per g for 2g range (32768/2)
#define BALANCE_PITCH -5.0f // Target pitch angle for balancing
#define FORWARD_TILT_PITCH -24.0f // Pitch angle to lean forward when moving
#define TURN_OFFSET 90.0f // Fixed turn speed offset (percentage)
#define FULL_TURN_THRESHOLD 70.0f // Degrees to turn for 90-degree turn

extern volatile bool pid_enabled;

typedef enum {
    TURN_OFF = 0,
    TURN_LEFT,
    TURN_RIGHT
} turn_state_t;

extern volatile turn_state_t turn_state;

typedef struct {
    int16_t gyro_rt;
    int16_t ax;
} pid_input_t;

typedef struct {
    int32_t pitch_int;
    int32_t pitch;
} pid_state_t;

void task_pid_init(void);
void task_pid_update(pid_state_t *s, int16_t gyro_rt, int16_t ax);
void task_pid_set_desired_pitch(float desired_pitch);
void task_pid_start_turn_left(void);
bool task_pid_is_turn_complete(void);

#endif