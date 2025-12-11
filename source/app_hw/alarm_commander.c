/**
 * @file alarm_commander.c
 * @brief Alarm commander task that coordinates alarm events and responses
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "alarm_commander.h"
#include "task_console.h"
#include "task_pid.h"
#include "task_amplifier.h"
#include "task_rtc.h"
#include "task_tof.h"
#include "pins.h"
#include <stdio.h>
#include <string.h>

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_alarm_commander(void *param);
static void stop_button_isr(void *callback_arg, cyhal_gpio_event_t event);
static BaseType_t cli_turn_90(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Event group for alarm commander */
EventGroupHandle_t eg_alarm_commander;

/* GPIO callback data structure */
static cyhal_gpio_callback_data_t stop_button_callback_data;

/******************************************************************************/
/* CLI Command Definitions                                                    */
/******************************************************************************/
static const CLI_Command_Definition_t cmd_turn_90 = {
	"turn_90",
	"turn_90\r\n    Turn 90 degrees left\r\n",
	cli_turn_90,
	0
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/**
 * @brief
 * Task that monitors alarm events and coordinates responses
 * @param param
 * Unused
 */
static void task_alarm_commander(void *param)
{
	EventBits_t event_bits;

	while (1)
	{
		/* Wait for notification */
		xEventGroupWaitBits(
			eg_alarm_commander,
			ALARM_CMD_BIT_NOTIFY,
			pdTRUE,  /* Clear NOTIFY bit on exit */
			pdFALSE,
			portMAX_DELAY);

		/* Read current state of event bits */
		event_bits = xEventGroupGetBits(eg_alarm_commander);

		/* Check which bits are set */
		if ((event_bits & ALARM_CMD_BIT_ALARM) && (event_bits & ALARM_CMD_BIT_OBJECT))
		{
			// Turn left using non-blocking turn - wait for gyro to reach 90 degrees
			task_pid_set_desired_pitch(BALANCE_PITCH);
			vTaskDelay(500);
			
			// Start the turn and wait for completion based on yaw angle
			task_pid_start_turn_left();
			while (!task_pid_is_turn_complete())
			{
				// Check if alarm was cancelled by button press
				EventBits_t current_bits = xEventGroupGetBits(eg_alarm_commander);
				if (!(current_bits & ALARM_CMD_BIT_ALARM))
				{
					// Alarm cancelled - stop turn immediately
					turn_state = TURN_OFF;
					break;
				}
				vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
			}

			xEventGroupClearBits(eg_alarm_commander, ALARM_CMD_BIT_OBJECT);
			task_tof_reset_object_state();
            xEventGroupSetBits(eg_alarm_commander, ALARM_CMD_BIT_NOTIFY);

			vTaskDelay(500);

            /* Start playing the alarm sound */
			amp_msg_t amp_msg;
			amp_msg.type = AMP_MSG_PLAY_TONE;
			amplifier_send_command(&amp_msg);
		}
		else if (event_bits & ALARM_CMD_BIT_ALARM)
		{
			/* MOVE FORWARD: Set the desired pitch angle to -20 degrees */
			task_pid_set_desired_pitch(FORWARD_TILT_PITCH);

			/* Start playing the alarm sound */
			amp_msg_t amp_msg;
			amp_msg.type = AMP_MSG_PLAY_TONE;
			amplifier_send_command(&amp_msg);
		}
		else
		{
			/* STOP: set desired pitch angle to 0 degrees to balance */
            task_pid_set_desired_pitch(BALANCE_PITCH);

            /* Stop playing the alarm sound */
			amp_msg_t amp_msg;
			amp_msg.type = AMP_MSG_STOP_TONE;
			amplifier_send_command(&amp_msg);
		}
	}
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief
 * Initializes software resources related to the alarm commander task.
 */
void task_alarm_commander_init(void)
{
	cy_rslt_t rslt;

	/* Create the event group for alarm commander */
	eg_alarm_commander = xEventGroupCreate();

	/* Initialize stop button GPIO */
	rslt = cyhal_gpio_init(STOP_BUTTON_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);
	CY_ASSERT(rslt == CY_RSLT_SUCCESS);

	/* Configure interrupt callback data */
	stop_button_callback_data.callback = stop_button_isr;
	stop_button_callback_data.callback_arg = NULL;

	/* Register interrupt callback */
	cyhal_gpio_register_callback(STOP_BUTTON_PIN, &stop_button_callback_data);

	/* Enable interrupt on falling edge (button press) */
	cyhal_gpio_enable_event(STOP_BUTTON_PIN, CYHAL_GPIO_IRQ_FALL, 3, true);

	/* Register CLI commands */
	FreeRTOS_CLIRegisterCommand(&cmd_turn_90);

	/* Create the alarm commander task */
	xTaskCreate(
		task_alarm_commander,
		"Task Alarm Commander",
		TASK_ALARM_COMMANDER_STACK_SIZE,
		NULL,
		TASK_ALARM_COMMANDER_PRIORITY,
		NULL);
}

/******************************************************************************/
/* GPIO Interrupt Handler                                                     */
/******************************************************************************/

/**
 * @brief
 * GPIO interrupt callback for stop button press
 * @param callback_arg
 * Unused callback argument
 * @param event
 * GPIO event type (falling edge)
 */
static void stop_button_isr(void *callback_arg, cyhal_gpio_event_t event)
{
	(void)callback_arg;
	(void)event;

	/* Call the stop alarm function from ISR context */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	int hours, minutes;

	/* Modify alarm time to be one minute less to avoid it going off again*/
	if (alarm_time_mutex != NULL)
	{
		/* Try to take semaphore from ISR - if we can't get it immediately, skip */
		if (xSemaphoreTakeFromISR(alarm_time_mutex, &xHigherPriorityTaskWoken) == pdTRUE)
		{
			/* Parse current alarm time */
			if (sscanf(alarm_time_str, "%d:%d", &hours, &minutes) == 2)
			{
				/* Decrement by one minute */
				minutes--;
				if (minutes < 0)
				{
					minutes = 59;
					hours--;
					if (hours < 0)
					{
						hours = 23;
					}
				}
				
				/* Update alarm_time_str */
				sprintf(alarm_time_str, "%02d:%02d", hours, minutes);
			}
			
			xSemaphoreGiveFromISR(alarm_time_mutex, &xHigherPriorityTaskWoken);
		}
	}

	/* Clear the ALARM bit and notify the task using ISR-safe functions */
	xEventGroupClearBitsFromISR(eg_alarm_commander, ALARM_CMD_BIT_ALARM);
	xEventGroupSetBitsFromISR(eg_alarm_commander, ALARM_CMD_BIT_NOTIFY, &xHigherPriorityTaskWoken);
	
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************/
/* CLI Command Implementations                                                */
/******************************************************************************/

/**
 * @brief
 * CLI command to turn 90 degrees left
 * @param pcWriteBuffer
 * Buffer to write response message
 * @param xWriteBufferLen
 * Length of write buffer
 * @param pcCommandString
 * Command string (unused)
 * @return pdFALSE (command complete)
 */
static BaseType_t cli_turn_90(char *pcWriteBuffer, size_t xWriteBufferLen,
                               const char *pcCommandString)
{
	(void)pcCommandString;
	(void)xWriteBufferLen;

	/* Start the turn and wait for completion based on yaw angle */
	task_pid_start_turn_left();
	while (!task_pid_is_turn_complete())
	{
		vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
	}

	return pdFALSE;
}
