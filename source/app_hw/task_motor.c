/**
 * @file task_motor.c
 * @author Mannan Jindal
 * @brief
 * DC Motor control using FreeRTOS task and DRV8251 H-Bridge driver.
 *
 * The FreeRTOS task receives commands from a FreeRTOS queue to control
 * motor direction and speed. 
 *
 * A FreeRTOS CLI command allows the user to control the motor from a serial console.
 *
 * @version 0.1
 * @date 2025-09-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "task_motor.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dc_motor.h"
#include "encoder.h"   // <-- NEW

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_motor(void *param);


static BaseType_t cli_handler_motor(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands to control the motor */
QueueHandle_t q_motor;

/* PWM objects for motor control */
static cyhal_pwm_t pwm_motor_1;
static cyhal_pwm_t pwm_motor_2;
static cyhal_pwm_t pwm_motor_3;
static cyhal_pwm_t pwm_motor_4;

/* Function to get PWM objects for direct motor control */
void task_motor_get_pwm_objects(cyhal_pwm_t **pwm1, cyhal_pwm_t **pwm2, cyhal_pwm_t **pwm3, cyhal_pwm_t **pwm4)
{
    *pwm1 = &pwm_motor_1;
    *pwm2 = &pwm_motor_2;
    *pwm3 = &pwm_motor_3;
    *pwm4 = &pwm_motor_4;
}

/* The CLI command definition for the motor command */
static const CLI_Command_Definition_t xMotor =
    {
        "motor",                                                    /* command text */
        "\r\nmotor <direction> <speed>\r\n"
        "  direction: forward, reverse, brake, coast, stop\r\n"
        "  speed: 0-100 (percentage)\r\n",                         /* command help text */
        cli_handler_motor,                                          /* The function to run. */
        2                                                           /* The user can enter 2 parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/* This task receives commands from the motor message queue to control
   motor direction and speed */
static void task_motor(void *param)
{
    motor_message_t message;

    /* Suppress warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for (;;)
    {
        // Check the Queue. If nothing was in the queue, we should return pdFALSE
        xQueueReceive(q_motor, &message, portMAX_DELAY);

        /* Set motor direction and speed */
        dc_motor_set_speed(message.speed, message.direction, &pwm_motor_1, &pwm_motor_2, &pwm_motor_3, &pwm_motor_4);

        /* Tell the encoder controller about the new base settings */
        encoder_set_base_direction(message.direction);  // <-- NEW
        encoder_set_base_speed(message.speed);          // <-- NEW
    }
}

/* FreeRTOS CLI Handler for the 'motor' command */
static BaseType_t cli_handler_motor(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    const char *pcParameter1, *pcParameter2;
    motor_message_t motor_msg;
    BaseType_t xParameterStringLength1, xParameterStringLength2, xReturn;
    char direction_str[16];
    int speed_value;

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    /* Obtain the first parameter (direction). */
    pcParameter1 = FreeRTOS_CLIGetParameter(
        pcCommandString,         /* The command string itself. */
        1,                       /* Return the 1st parameter. */
        &xParameterStringLength1 /* Store the parameter string length. */
    );
    configASSERT(pcParameter1);

    /* Obtain the second parameter (speed). */
    pcParameter2 = FreeRTOS_CLIGetParameter(
        pcCommandString,         /* The command string itself. */
        2,                       /* Return the 2nd parameter. */
        &xParameterStringLength2 /* Store the parameter string length. */
    );
    configASSERT(pcParameter2);

    /* Sanity check something was returned. */
    if (pcParameter1 == NULL || pcParameter2 == NULL)
    {
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        sprintf(pcWriteBuffer, "Error... missing parameters\n\r");
        return pdFALSE;
    }

    /* Copy parameters to local buffers */
    memset(direction_str, 0x00, sizeof(direction_str));
    strncat(direction_str, pcParameter1, xParameterStringLength1);

    /* Parse direction parameter */
    if (strcmp(direction_str, "forward") == 0)
    {
        motor_msg.direction = DC_MOTOR_FORWARD;
    }
    else if (strcmp(direction_str, "reverse") == 0)
    {
        motor_msg.direction = DC_MOTOR_REVERSE;
    }
    else if (strcmp(direction_str, "right") == 0)
    {
        motor_msg.direction = DC_MOTOR_RIGHT;
    }
    else if (strcmp(direction_str, "left") == 0)
    {
        motor_msg.direction = DC_MOTOR_LEFT;
    }
    else if (strcmp(direction_str, "stop") == 0)
    {
        motor_msg.direction = DC_MOTOR_STOP;
    }
    else
    {
        /* Return a string indicating an invalid direction parameter. */
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        sprintf(pcWriteBuffer, "Error... invalid direction parameter\n\r");
        return pdFALSE;
    }

    /* Parse speed parameter */
    speed_value = atoi(pcParameter2);
    if (speed_value < 0 || speed_value > 100)
    {
        /* Return a string indicating an invalid speed parameter. */
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        sprintf(pcWriteBuffer, "Error... speed must be 0-100\n\r");
        return pdFALSE;
    }

    motor_msg.speed = (uint8_t)speed_value;

    /* Send the message to the motor task */
    xQueueSendToBack(q_motor, &motor_msg, portMAX_DELAY);

    /* Provide feedback to user */
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Motor set to %s at %d speed\n\r", direction_str, speed_value);

    /* Indicate that the command has completed */
    xReturn = pdFALSE;

    return xReturn;
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/
void task_motor_init(void)
{
    cy_rslt_t rslt;

    /* Initialize the DC motor hardware */
    rslt = dc_motor_init();
    if(rslt != CY_RSLT_SUCCESS)
    {
        // Handle error
        CY_ASSERT(0);
    }
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    /* Initialize PWM objects for motor control */
    rslt = dc_motor_pwm_init(&pwm_motor_1, &pwm_motor_2, &pwm_motor_3, &pwm_motor_4);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    /* NEW: Give encoder controller access to the four PWM objects */
    encoder_register_pwm_objects(&pwm_motor_1, &pwm_motor_2, &pwm_motor_3, &pwm_motor_4);  // <-- NEW

    /* Create the Queue used to control the motor */
    q_motor = xQueueCreate(1, sizeof(motor_message_t));

    /* Register the CLI command */
    FreeRTOS_CLIRegisterCommand(&xMotor);

    /* Create the task that will control the motor */
    xTaskCreate(
        task_motor,
        "Task_Motor",
        configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 6,
        NULL);

    /* Start ISR-based encoder edge counting (aggregates per 0.5 s) */
    encoder_start_edge_counting();  // <-- already present
}
