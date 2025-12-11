#include "encoder.h"
#include "cyhal_gpio.h"
#include "task_console.h"

/* ====== Internal state ====== */
static volatile int32_t s_m1_edges = 0;      // aggregated edges in current 0.5 s window
static volatile int32_t s_m2_edges = 0;
static int32_t         s_m1_last = 0;        // latched counts from the last window (for debugging)
static int32_t         s_m2_last = 0;

static uint8_t s_base_speed = 0;             // commanded (CLI) speed, 0–100
static dc_motor_direction_t s_dir = DC_MOTOR_STOP;

/* Independent correction offsets applied on top of base speed */
static int8_t s_m1_offset = 0;               // signed, range-limited so base+offset always 0–100
static int8_t s_m2_offset = 0;

/* The 4 PWM objects driving IN1/IN2 for both motors */
static cyhal_pwm_t *s_pwm_m1_in1 = NULL;
static cyhal_pwm_t *s_pwm_m1_in2 = NULL;
static cyhal_pwm_t *s_pwm_m2_in1 = NULL;
static cyhal_pwm_t *s_pwm_m2_in2 = NULL;

/* One callback data object per encoder pin (must persist in memory) */
static cyhal_gpio_callback_data_t s_cb_m1_a;
static cyhal_gpio_callback_data_t s_cb_m1_b;
static cyhal_gpio_callback_data_t s_cb_m2_a;
static cyhal_gpio_callback_data_t s_cb_m2_b;


/* FreeRTOS control task handle (not used externally) */
static TaskHandle_t s_ctrl_task = NULL;



/* ====== Helpers ====== */
static inline uint8_t clamp_u8(int32_t v)
{
    if (v < 0)   return 0;
    if (v > 100) return 100;
    return (uint8_t)v;
}

/* Apply per-motor duty depending on direction without fighting the main API.
   We do not stop/start PWM — HAL supports duty updates on the fly. */
static void apply_motor_duties(uint8_t spd_m1, uint8_t spd_m2)
{
    if (!s_pwm_m1_in1 || !s_pwm_m1_in2 || !s_pwm_m2_in1 || !s_pwm_m2_in2) return;

    switch (s_dir)
    {
        case DC_MOTOR_FORWARD:
            /* IN1 = PWM, IN2 = 0 for both motors */
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in1, spd_m1, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in2, 0,     1000);

            cyhal_pwm_set_duty_cycle(s_pwm_m2_in1, spd_m2, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in2, 0,     1000);
            break;

        case DC_MOTOR_REVERSE:
            /* IN1 = 0, IN2 = PWM for both motors */
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in1, 0,     1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in2, spd_m1, 1000);

            cyhal_pwm_set_duty_cycle(s_pwm_m2_in1, 0,     1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in2, spd_m2, 1000);
            break;

        case DC_MOTOR_LEFT:
            /* Left motor reverse, right motor forward */
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in1, 0, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in2, spd_m1, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in1, spd_m2, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in2, 0, 1000);
            break;

        case DC_MOTOR_RIGHT:
            /* Left motor forward, right motor reverse */
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in1, spd_m1, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in2, 0, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in1, 0, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in2, spd_m2, 1000);
            break;

        case DC_MOTOR_STOP:
        default:
            /* Low-low (coast/stop); keep equal */
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in1, 0, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m1_in2, 0, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in1, 0, 1000);
            cyhal_pwm_set_duty_cycle(s_pwm_m2_in2, 0, 1000);
            break;
    }
}

/* Simple PI-less balancing: proportional nudge + deadband + per-update clamp */
static void encoder_adjust_speeds(int32_t m1_count, int32_t m2_count)
{
    const int32_t deadband = 5;      // ignore tiny differences in 0.5 s window
    const int32_t max_step = 50;      // max +/- change (percent) per 0.5 s step
    const float   kP       = 0.10f;  // proportional gain (percent per count)

    int32_t diff = m1_count - m2_count; /* >0 => Motor1 produced more counts */

    if (diff > deadband)
    {
        /* Motor2 is lagging => increase M2 or decrease M1 */
        int32_t step = (int32_t)(kP * diff);
        if (step < 1) step = 1;
        if (step > max_step) step = max_step;

        s_m2_offset = (int8_t)clamp_u8((int32_t)s_base_speed + s_m2_offset + step) - (int8_t)s_base_speed;
        s_m1_offset = (int8_t)clamp_u8((int32_t)s_base_speed + s_m1_offset - step) - (int8_t)s_base_speed;
    }
    else if (diff < -deadband)
    {
        /* Motor1 is lagging => increase M1 or decrease M2 */
        int32_t step = (int32_t)(kP * (-diff));
        if (step < 1) step = 1;
        if (step > max_step) step = max_step;

        s_m1_offset = (int8_t)clamp_u8((int32_t)s_base_speed + s_m1_offset + step) - (int8_t)s_base_speed;
        s_m2_offset = (int8_t)clamp_u8((int32_t)s_base_speed + s_m2_offset - step) - (int8_t)s_base_speed;
    }
    else
    {
        /* In deadband: gently decay offsets back toward 0 so motors converge */
        if      (s_m1_offset > 0) s_m1_offset--;
        else if (s_m1_offset < 0) s_m1_offset++;

        if      (s_m2_offset > 0) s_m2_offset--;
        else if (s_m2_offset < 0) s_m2_offset++;
    }

    uint8_t spd_m1 = clamp_u8((int32_t)s_base_speed + s_m1_offset);
    uint8_t spd_m2 = clamp_u8((int32_t)s_base_speed + s_m2_offset);

    apply_motor_duties(spd_m1, spd_m2);
}

/* ====== GPIO ISR callback: increments the counter passed as arg ====== */
static void enc_gpio_isr(void *arg, cyhal_gpio_event_t event)
{
    (void)event;
    volatile int32_t *counter = (volatile int32_t *)arg;
    (*counter)++;  /* count both edges */
}


static uint32_t print_div = 0;

/* ====== Control task: runs every 500 ms ====== */
static void encoder_control_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(500);

    for (;;)
    {
        vTaskDelay(period);

        taskENTER_CRITICAL();
        int32_t c1 = s_m1_edges;
        int32_t c2 = s_m2_edges;
        s_m1_edges = 0;
        s_m2_edges = 0;
        s_m1_last = c1;
        s_m2_last = c2;
        taskEXIT_CRITICAL();


        /* Only balance in motion (forward or reverse) */
        if (s_dir == DC_MOTOR_FORWARD || s_dir == DC_MOTOR_REVERSE)
        {
            encoder_adjust_speeds(c1, c2);
        }
        else
        {
            /* Not moving — keep offsets zeroed, ensure outputs reflect base state */
            s_m1_offset = 0;
            s_m2_offset = 0;
            apply_motor_duties(s_base_speed, s_base_speed);
        }
    }
}

/* ====== Public API ====== */
void encoder_register_pwm_objects(cyhal_pwm_t *pwm_m1_in1,
                                  cyhal_pwm_t *pwm_m1_in2,
                                  cyhal_pwm_t *pwm_m2_in1,
                                  cyhal_pwm_t *pwm_m2_in2)
{
    s_pwm_m1_in1 = pwm_m1_in1;
    s_pwm_m1_in2 = pwm_m1_in2;
    s_pwm_m2_in1 = pwm_m2_in1;
    s_pwm_m2_in2 = pwm_m2_in2;
}

void encoder_set_base_speed(uint8_t percent)
{
    s_base_speed = clamp_u8(percent);
    /* also re-apply immediately to keep visible when direction already set */
    //apply_motor_duties(s_base_speed, s_base_speed);
}

void encoder_set_base_direction(dc_motor_direction_t dir)
{
    s_dir = dir;
    /* on direction change, reset offsets to avoid a jump in wrong pins */
    s_m1_offset = 0;
    s_m2_offset = 0;
    apply_motor_duties(s_base_speed, s_base_speed);
}

void encoder_get_last_counts(int32_t *m1_count, int32_t *m2_count)
{
    if (m1_count) *m1_count = s_m1_last;
    if (m2_count) *m2_count = s_m2_last;
}

void encoder_start_edge_counting(void)
{
    /* Configure four encoder pins as inputs with both-edge interrupts */
    cyhal_gpio_init(MOTOR_ENC1_1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    cyhal_gpio_init(MOTOR_ENC1_2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    cyhal_gpio_init(MOTOR_ENC2_1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    cyhal_gpio_init(MOTOR_ENC2_2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

    /* Prepare callback data for each pin (HAL v2.7.x expects this struct) */
    s_cb_m1_a.callback     = enc_gpio_isr;
    s_cb_m1_a.callback_arg = (void *)&s_m1_edges;

    s_cb_m1_b.callback     = enc_gpio_isr;
    s_cb_m1_b.callback_arg = (void *)&s_m1_edges;

    s_cb_m2_a.callback     = enc_gpio_isr;
    s_cb_m2_a.callback_arg = (void *)&s_m2_edges;

    s_cb_m2_b.callback     = enc_gpio_isr;
    s_cb_m2_b.callback_arg = (void *)&s_m2_edges;

    /* Register callbacks */
    cyhal_gpio_register_callback(MOTOR_ENC1_1, &s_cb_m1_a);
    cyhal_gpio_register_callback(MOTOR_ENC1_2, &s_cb_m1_b);
    cyhal_gpio_register_callback(MOTOR_ENC2_1, &s_cb_m2_a);
    cyhal_gpio_register_callback(MOTOR_ENC2_2, &s_cb_m2_b);

    /* Enable both-edge interrupts; priority 3 matches your other uses */
    const uint8_t prio = 3;
    cyhal_gpio_enable_event(MOTOR_ENC1_1, CYHAL_GPIO_IRQ_BOTH, prio, true);
    cyhal_gpio_enable_event(MOTOR_ENC1_2, CYHAL_GPIO_IRQ_BOTH, prio, true);
    cyhal_gpio_enable_event(MOTOR_ENC2_1, CYHAL_GPIO_IRQ_BOTH, prio, true);
    cyhal_gpio_enable_event(MOTOR_ENC2_2, CYHAL_GPIO_IRQ_BOTH, prio, true);

    /* Spawn control loop if not already running */
    if (s_ctrl_task == NULL)
    {
        xTaskCreate(encoder_control_task,
                    "EncCtrl",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    &s_ctrl_task);
    }

}
