/**
 * @file io_expander.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "task_rtc.h"
#include "task_console.h"
#include "task_amplifier.h"
#include "alarm_commander.h"
#include <stdint.h>

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_rtc_cli(void *param);
static void task_rtc(void *param);

static BaseType_t cli_handler_rtc_time(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands used to rtc */
QueueHandle_t q_rtc_req;
QueueHandle_t q_rtc_rsp;

/* Event group for alarm events */
EventGroupHandle_t eg_rtc_alarm;

/* Alarm time in HH:MM string format */
char alarm_time_str[8] = "99:00";

/* Mutex to protect alarm_time_str from race conditions */
SemaphoreHandle_t alarm_time_mutex = NULL;

/* The CLI command definition for the rtc command */
static const CLI_Command_Definition_t cmd_rtc_time =
	{
		"rtc",							   /* command text */
		"\r\nrtc <set|read|alarm> <hour> <minute> <second>\r\n", /* command help text */
		cli_handler_rtc_time,				   /* The function to run. */
		-1								   /* The user can enter any number of parameters */
};


// TODO: Implement CLI support for date setting and getting
// static const CLI_Command_Definition_t cmd_rtc_date =
// 	{
// 		"rtc",							   /* command text */
// 		"\r\nrtc <set|read> <month> <day> <year> <hour> <minute> <second>\r\n", /* command help text */
// 		cli_handler_rtc,				   /* The function to run. */
// 		-1								   /* The user can enter any number of parameters */
// };

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/** Convert a decimal value to BCD (Binary Coded Decimal)
 *
 * @param decimal The decimal value to convert (0-99)
 * @return The BCD encoded value
 *
 */
uint8_t decimal_to_bcd(uint8_t decimal)
{
	uint8_t bcd = 0;
	bcd = ((decimal / 10) << 4) | (decimal % 10);
	return bcd;
}

/** Convert a BCD (Binary Coded Decimal) value to decimal
 *
 * @param bcd The BCD encoded value
 * @return The decimal value
 *
 */
uint8_t bcd_to_decimal(uint8_t bcd)
{
	uint8_t decimal = 0;
	decimal = ((bcd >> 4) * 10) + (bcd & 0x0F);
	return decimal;
}

/** Write a register on the BQ32002DR
 *
 * @param reg The reg address to read
 * @param value The value to be written
 *
 */
static void rtc_write_reg(uint8_t reg, uint8_t value)
{
	cy_rslt_t rslt;
	uint8_t write_buffer[2];

	write_buffer[0] = reg;
	write_buffer[1] = value;

	/* There may be multiple I2C devices on the same bus, so we must take
	   the semaphore used to ensure mutual exclusion before we begin any
	   I2C transactions
	 */
	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	/* Use cyhal_i2c_master_write to write the required data to the device. */
	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,		  // I2C Object
		BQ32002DR_SUBORDINATE_ADDR, // I2C Address
		write_buffer,			  // Array of data to write
		2,						  // Number of bytes to write
		0,						  // Block until completed
		true);					  // Generate Stop Condition

	if (rslt != CY_RSLT_SUCCESS)
	{
		task_print_error("Error writing to the RTC");
	}

	/* Give up control of the I2C bus */
	xSemaphoreGive(Semaphore_I2C);
}

/** Read a register on the BQ32002DR
 *
 * @param reg The reg address to read
 *
 */
static uint8_t rtc_read_reg(uint8_t reg)
{
	cy_rslt_t rslt;

	uint8_t write_buffer[1];
	uint8_t read_buffer[1];

	write_buffer[0] = reg;

	/* There may be multiple I2C devices on the same bus, so we must take
	   the semaphore used to ensure mutual exclusion before we begin any
	   I2C transactions
	 */
	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	/* Use cyhal_i2c_master_write to write the required data to the device. */
	/* Send the register address, do not generate a stop condition.  This will result in */
	/* a restart condition. */
	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,
		BQ32002DR_SUBORDINATE_ADDR, // I2C Address
		write_buffer,			  // Array of data to write
		1,						  // Number of bytes to write
		0,						  // Block until completed
		false);					  // Do NOT generate Stop Condition

	if (rslt != CY_RSLT_SUCCESS)
	{
		task_print_error("Error writing reg address to RTC");
	}
	else
	{

		/* Use cyhal_i2c_master_read to read the required data from the device. */
		// The register address has already been set in the write above, so read a single byte
		// of data.
		rslt = cyhal_i2c_master_read(
			&i2c_master_obj,		  // I2C Object
			BQ32002DR_SUBORDINATE_ADDR, // I2C Address
			read_buffer,			  // Read Buffer
			1,						  // Number of bytes to read
			0,						  // Block until completed
			true);					  // Generate Stop Condition

		if (rslt != CY_RSLT_SUCCESS)
		{
			task_print_error("Error Reading from the RTC");
			read_buffer[0] = 0xFF;
		}
	}

	/* Give up control of the I2C bus */
	xSemaphoreGive(Semaphore_I2C);

	return read_buffer[0];
}

/**
 * @brief
 * This function parses the list of parameters to determine the type of
 * operation that is requested. The 'operation' field data structure will
 * be updated.
 *
 * The value will only be used on a write operation.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @param packet
 * A pointer to the data structure that will be sent to the io expander task.
 * @return BaseType_t
 * pdPASS if a valid operation is found.
 * pdFAIL if an invalid operation is found.
 */
static BaseType_t cli_handler_rtc_time_get_operation(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	rtc_packet_t *packet)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;
	BaseType_t xReturn;

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,		/* The command string itself. */
		1,						/* Return the 1st parameter. */
		&lParameterStringLength /* Store the parameter string length. */
	);
	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, pcParameter, lParameterStringLength);

	/* Verify that the first parameter is either set or read */
	if ((strcmp(pcWriteBuffer, "set")) == 0)
	{
		packet->operation = RTC_OP_SET_TIME;
		xReturn = pdTRUE;
	}
	else if ((strcmp(pcWriteBuffer, "read")) == 0)
	{
		packet->operation = RTC_OP_READ_TIME;
		xReturn = pdTRUE;
	}
	else if ((strcmp(pcWriteBuffer, "alarm")) == 0)
	{
		packet->operation = RTC_OP_SET_ALARM;
		xReturn = pdTRUE;
	}
	else
	{
		/* The first parameter is invalid*/
		packet->operation = RTC_OP_INVALID;
		xReturn = pdFAIL;
	}

	return xReturn;
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
 * @param packet
 * A pointer to the data structure that will be sent to the rtc task.
 * @return BaseType_t
 * pdPASS if a valid value was found.
 * pdFAIL if the value is invalid
 */
static BaseType_t cli_handler_rtc_get_time(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	rtc_packet_t *packet)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;
	BaseType_t xReturn;
	char *end_ptr;
	uint32_t result;

	for(uint8_t i = 2; i < 5; i++)
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			i,						/* Return the 2nd, 3rd, and 4th parameters. */
			&lParameterStringLength /* Store the parameter string length. */
		);
		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		strncat(pcWriteBuffer, pcParameter, lParameterStringLength);

		/* Convert the stirng to an address */
		result = strtol(pcWriteBuffer, &end_ptr, 10);
		if (*end_ptr != '\0')
		{
			xReturn = pdFAIL;
			break;
		}
		else
		{
			switch(i)
			{
				case 2:
					packet->time_date.hours = decimal_to_bcd((uint8_t)result);
					break;
				case 3:
					packet->time_date.minutes = decimal_to_bcd((uint8_t)result);
					break;
				case 4:
					packet->time_date.seconds = decimal_to_bcd((uint8_t)result);
					break;
				default:
					break;
			}
			xReturn = pdPASS;
		}
	}
	return xReturn;
}

/**
 * @brief
 * FreeRTOS CLI Handler for the 'ioxp' command.
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
static BaseType_t cli_handler_rtc_time(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString)
{
	BaseType_t xReturn;
	rtc_packet_t rtc_packet;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Get the ioxp operation type */
	xReturn = cli_handler_rtc_time_get_operation(
		pcWriteBuffer,
		xWriteBufferLen,
		pcCommandString,
		&rtc_packet);

	/* Return if the user entered an invalid operation*/
	if (xReturn == pdFAIL)
	{
		/* Clear the return string */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid RTC Operation");
		return xReturn;
	}

	// Set the time
	if(rtc_packet.operation == RTC_OP_SET_TIME)
	{
		/* Indicate which queue to return the data to */
		rtc_packet.return_queue = q_rtc_rsp;

		/* Get the time to be set */
		xReturn = cli_handler_rtc_get_time(
			pcWriteBuffer,
			xWriteBufferLen,
			pcCommandString,
			&rtc_packet);

		/* Return if the user entered an invalid operation*/
		if (xReturn == pdFAIL)
		{
			/* Clear the return string */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			sprintf(pcWriteBuffer, "\n\r\tInvalid RTC Time");
			return xReturn;
		}

		/* Send the packet to the rtc request queue  */
		xQueueSend(q_rtc_req, &rtc_packet, portMAX_DELAY);

		/* Return an empty string since there is nothing to return on a write */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);

		xReturn = pdFAIL;
		return xReturn;
	}
	// Read the time
	else if(rtc_packet.operation == RTC_OP_READ_TIME)
	{
		/* Indicate which queue to return the data to */
		rtc_packet.return_queue = q_rtc_rsp;

		/* Send the packet to the rtc request queue  */
		xQueueSend(q_rtc_req, &rtc_packet, portMAX_DELAY);

		/* Wait for the data to be returned */
		xQueueReceive(q_rtc_rsp, &rtc_packet, portMAX_DELAY);

		/* Return the value that was read as a string. */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\t%02u:%02u:%02u",
			rtc_packet.time_date.hours,
			rtc_packet.time_date.minutes,
			rtc_packet.time_date.seconds);

		/* Indicate to the main parser that the command was completed */
		xReturn = pdFAIL;
		return xReturn;
	}
	else if(rtc_packet.operation == RTC_OP_SET_ALARM)
	{
		/* Get the time to be set */
		xReturn = cli_handler_rtc_get_time(
			pcWriteBuffer,
			xWriteBufferLen,
			pcCommandString,
			&rtc_packet);

		/* Return if the user entered an invalid operation*/
		if (xReturn == pdFAIL)
		{
		/* Clear the return string */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid RTC Time");
		return xReturn;
	}

	/* Update the alarm time string with mutex protection */
	if (alarm_time_mutex != NULL)
	{
		xSemaphoreTake(alarm_time_mutex, portMAX_DELAY);
		sprintf(alarm_time_str, "%02u:%02u",
			bcd_to_decimal(rtc_packet.time_date.hours),
			bcd_to_decimal(rtc_packet.time_date.minutes));
		xSemaphoreGive(alarm_time_mutex);
	}

	/* Return an empty string since there is nothing to return on a write */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);

		xReturn = pdFAIL;
		return xReturn;
	}
	else
	{
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid RTC Operation");
		return pdFAIL;
	}
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
void task_rtc_cli(void *param)
{
	rtc_packet_t packet;

	while (1)
	{
		/* Wait for a message */
		xQueueReceive(q_rtc_req, &packet, portMAX_DELAY);

		if(packet.operation == RTC_OP_SET_TIME){
			/* Write the time to the RTC */
			rtc_write_reg(RTC_REG_ADDR_SECONDS, packet.time_date.seconds);
			rtc_write_reg(RTC_REG_ADDR_MINUTES, packet.time_date.minutes);
			rtc_write_reg(RTC_REG_ADDR_HOURS, packet.time_date.hours);
		}
		else if(packet.operation == RTC_OP_READ_TIME){
			/* Read the time from the RTC, mask off control bits */
			packet.time_date.seconds = bcd_to_decimal(rtc_read_reg(RTC_REG_ADDR_SECONDS) & 0x7F);
			packet.time_date.minutes = bcd_to_decimal(rtc_read_reg(RTC_REG_ADDR_MINUTES) & 0x7F);
			packet.time_date.hours   = bcd_to_decimal(rtc_read_reg(RTC_REG_ADDR_HOURS) & 0x3F);

			/*
			 * Send the data back to the task that made requested to read
			 * the data.
			 *
			 * The task that requests the data MUST be sure
			 * to set the return_queue field of the struct to an initialized
			 * queue.  See cli_handler_rtc() above for an example of how to send a
			 * read request and then wait for the response.
			 */
			xQueueSend(packet.return_queue, &packet, portMAX_DELAY);
		}
	}
}


/**
 * @brief
 * Task used to check for alarm and update LCD with current time
 * @param param
 * Unused
 */
void task_rtc(void *param)
{
	uint8_t current_hours;
	uint8_t current_minutes;
	char current_time_str[8] = "00:00";

	while (1)
	{
		/* Read the current time from the RTC */
		current_hours = bcd_to_decimal(rtc_read_reg(RTC_REG_ADDR_HOURS) & 0x3F);
		current_minutes = bcd_to_decimal(rtc_read_reg(RTC_REG_ADDR_MINUTES) & 0x7F);

	/* Update the current time string */
	sprintf(current_time_str, "%02u:%02u", current_hours, current_minutes);
	xQueueSend(q_lcd_req, current_time_str, portMAX_DELAY);

	/* Check if the current time matches the alarm time with mutex protection */
	if (alarm_time_mutex != NULL)
	{
		xSemaphoreTake(alarm_time_mutex, portMAX_DELAY);
		bool alarm_match = (strcmp(alarm_time_str, current_time_str) == 0);
		xSemaphoreGive(alarm_time_mutex);
		
		if (alarm_match)
		{
			/* Set the alarm triggered event bit */
			xEventGroupSetBits(eg_alarm_commander, ALARM_CMD_BIT_ALARM | ALARM_CMD_BIT_NOTIFY);
		}
	}

	/* Delay for a period before checking again */
	vTaskDelay(pdMS_TO_TICKS(ALARM_CHECK_INTERVAL_MS));
	}
}

/**
 * @brief
 * Initializes software resources related to the operation of
 * the IO Expander.  This function expects that the I2C bus had already
 * been initialized prior to the start of FreeRTOS.
 */
void task_rtc_init(void)
{
	/* Create the Queue used to control blinking of the status LED*/
	q_rtc_req = xQueueCreate(1, sizeof(rtc_packet_t));
	q_rtc_rsp = xQueueCreate(1, sizeof(rtc_packet_t));

	/* Create mutex to protect alarm_time_str */
	alarm_time_mutex = xSemaphoreCreateMutex();

	/* Register the CLI command */
	FreeRTOS_CLIRegisterCommand(&cmd_rtc_time);

	/* Create the task that will interface with CLI */
	xTaskCreate(
		task_rtc_cli,
		"Task CLI RTC",
		configMINIMAL_STACK_SIZE,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);

		/* Create the task that will check time and trigger alarm */
	xTaskCreate(
		task_rtc,
		"Task RTC Alarm",
		configMINIMAL_STACK_SIZE,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);
}
