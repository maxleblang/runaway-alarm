/**
 * @file servo.c
 * @author Max Leblang
 * @brief
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "task_servo.h"
#include "pins.h"
#include "pwm.h"

cyhal_pwm_t servo_pwm_obj;

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_servo(void *param);

static BaseType_t cli_handler_servo(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands used to servo */
QueueHandle_t q_servo_angle;

/* The CLI command definition for the ioxp command */
static const CLI_Command_Definition_t cmd_servo =
	{
		"servo",							   	/* command text */
		"\r\nservo <angle>\r\n", 			/* command help text */
		cli_handler_servo,				   /* The function to run. */
		1								
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/
static void set_servo_angle(uint32_t angle)
{
	float duty_cycle;

	/* Calculate the duty cycle based on the angle */
	// NOTE: Adjust these values based on your servo's specifications
	duty_cycle = ((float)angle * (8.0f / 180.0f)) + 3.0f;

	/* Set the new duty cycle while maintaining 50Hz frequency */
	cyhal_pwm_set_duty_cycle(&servo_pwm_obj, duty_cycle, PWM_SERVO_FREQ_HZ);
}


/**
 * @brief
 * This function parses the list of parameters for the 3rd parameter
 * and sets the 'value' field data structure
 *
 * The value will only be used on a write operation.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @param angle
 * A pointer to the variable to store the angle value
 * @return BaseType_t
 * pdPASS if a valid value was found.
 * pdFAIL if the value is invalid
 */
static BaseType_t cli_handler_servo_get_angle(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	uint32_t *angle)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;
	BaseType_t xReturn;
	char *end_ptr;
	uint32_t result;

	/* Obtain the register address string. */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,		/* The command string itself. */
		1,						/* Return the 3rd parameter. */
		&lParameterStringLength /* Store the parameter string length. */
	);
	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, pcParameter, lParameterStringLength);

	/* Convert the string to an angle (base 10) */
	result = strtol(pcWriteBuffer, &end_ptr, 10);
	if (*end_ptr != '\0')
	{
		*angle = 0;
		xReturn = pdFAIL;
	}
	else
	{
		*angle = (uint32_t)result;
		if (*angle > 180)
		{
			*angle = 0;
			xReturn = pdFAIL;
		}
		else
		{
			xReturn = pdPASS;
		}
	}

	return xReturn;
}

/**
 * @brief
 * FreeRTOS CLI Handler for the 'servo' command.
 *
 * This function will be executed from task_console_rx() when
 * FreeRTOS_CLIProcessCommand() is called.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @return BaseType_t
 * pdPASS if a valid value was found.
 * pdFAIL if the value is invalid
 */
static BaseType_t cli_handler_servo(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString)
{
	BaseType_t xReturn;
	uint32_t angle;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Get the servo angle */
	xReturn = cli_handler_servo_get_angle(
		pcWriteBuffer,
		xWriteBufferLen,
		pcCommandString,
		&angle);

	/* Return if the user entered an invalid operation*/
	if (xReturn == pdFAIL)
	{
		/* Clear the return string */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid servo angle");
		return xReturn;
	}

	xQueueSend(q_servo_angle, &angle, portMAX_DELAY);
	return pdFAIL;
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief
 * Task used to monitor the reception of command packets sent the io expander
 * @param param
 * Unused
 */
void task_servo(void *param)
{
	uint32_t angle;

	while (1)
	{
		/* Wait for a message */
		xQueueReceive(q_servo_angle, &angle, portMAX_DELAY);

		/* Set the servo angle */
		set_servo_angle(angle);
	}
}

/**
 * @brief
 * Initializes software resources related to the operation of
 * the IO Expander.  This function expects that the I2C bus had already
 * been initialized prior to the start of FreeRTOS.
 */
void task_servo_init(void)
{
	cy_rslt_t result;
	// Initialize PWM for Servo control here
	result = pwm_init(SERVO_PWM,
	             PWM_SERVO_FREQ_HZ,
	             PWM_INIT_DUTY_CYCLE,
	             &servo_pwm_obj);
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* Create the Queue used to control blinking of the status LED*/
	q_servo_angle = xQueueCreate(1, sizeof(uint32_t));
	
	/* Register the CLI command */
	FreeRTOS_CLIRegisterCommand(&cmd_servo);

	/* Create the task that will control the status LED */
	xTaskCreate(
		task_servo,
		"Task Servo",
		configMINIMAL_STACK_SIZE,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);
}
