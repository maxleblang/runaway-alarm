/**
 * @file task_lcd.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "task_lcd.h"
#include <stdint.h>
#include <string.h>

static const uint8_t lcd_row_offsets[LCD_NUM_ROWS] = {0x00u, 0x40u};

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_lcd(void *param);

static BaseType_t cli_handler_lcd(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString);

static void lcd_write_command(uint8_t cmd);
static void lcd_write_data(uint8_t data);
static void lcd_init_display(void);
static void lcd_set_cursor(uint8_t col, uint8_t row);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands used to lcd */
QueueHandle_t q_lcd_req;

/* The CLI command definition for the rtc command */
static const CLI_Command_Definition_t cmd_lcd =
	{
		"lcd",							   /* command text */
		"\r\nlcd <string>\r\n", /* command help text */
		cli_handler_lcd,				   /* The function to run. */
		-1								   /* The user can enter any number of parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/
static void lcd_write_raw(uint8_t value)
{
	cy_rslt_t rslt;

	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,
		CN0295D_SUBORDINATE_ADDR,
		&value,
		1,
		0,
		true);

	if (rslt != CY_RSLT_SUCCESS)
	{
		task_print_error("Error writing to the LCD");
	}

	xSemaphoreGive(Semaphore_I2C);
}

static void lcd_pulse_enable(uint8_t data)
{
	lcd_write_raw(data | LCD_CTRL_EN);
	vTaskDelay(pdMS_TO_TICKS(1));
	lcd_write_raw(data & (uint8_t)~LCD_CTRL_EN);
	vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
	uint8_t data = (uint8_t)((nibble & 0xF0u) | mode | LCD_CTRL_BL);
	lcd_write_raw(data);
	lcd_pulse_enable(data);
}

static void lcd_send(uint8_t value, uint8_t mode)
{
	lcd_write_nibble(value & 0xF0u, mode);
	lcd_write_nibble((uint8_t)((value << 4) & 0xF0u), mode);
}

/**
 * @brief
 * Helper function to write a command to the LCD
 * @param cmd
 * The command byte to send
 */
static void lcd_write_command(uint8_t cmd)
{
	lcd_send(cmd, 0x00u);
	if ((cmd == 0x01u) || (cmd == 0x02u))
	{
		vTaskDelay(pdMS_TO_TICKS(2));
	}
	else
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

/**
 * @brief
 * Helper function to write character data to the LCD
 * @param data
 * The character byte to display
 */
static void lcd_write_data(uint8_t data)
{
	lcd_send(data, LCD_CTRL_RS);
	vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * @brief
 * Initialize the LCD display with proper setup sequence
 */
static void lcd_set_cursor(uint8_t col, uint8_t row)
{
	if (row >= LCD_NUM_ROWS)
	{
		row = LCD_NUM_ROWS - 1u;
	}
	if (col >= LCD_NUM_COLS)
	{
		col = LCD_NUM_COLS - 1u;
	}
	lcd_write_command((uint8_t)(0x80u | (lcd_row_offsets[row] + col)));
}

static void lcd_init_display(void)
{
	/* Bring the HD44780 into 4-bit mode via the PCF8574 backpack */
	vTaskDelay(pdMS_TO_TICKS(50));
	lcd_write_nibble(0x30u, 0x00u);
	vTaskDelay(pdMS_TO_TICKS(5));
	lcd_write_nibble(0x30u, 0x00u);
	vTaskDelay(pdMS_TO_TICKS(1));
	lcd_write_nibble(0x30u, 0x00u);
	vTaskDelay(pdMS_TO_TICKS(1));
	lcd_write_nibble(0x20u, 0x00u);

	/* Function set: 4-bit, 2 line, 5x8 font */
	lcd_write_command(0x28u);
	/* Display off while we finish configuration */
	lcd_write_command(0x08u);
	/* Clear and set entry mode */
	lcd_write_command(0x01u);
	lcd_write_command(0x06u);
	/* Display on, cursor off */
	lcd_write_command(0x0Cu);
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
static BaseType_t cli_handler_lcd_get_string(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	char *string_buffer)
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
	strcpy(string_buffer, pcParameter);

	xReturn = pdPASS;
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
static BaseType_t cli_handler_lcd(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString)
{
	BaseType_t xReturn;
	char string_buffer[LCD_STRING_MAX_LEN];

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Get the ioxp operation type */
	xReturn = cli_handler_lcd_get_string(
		pcWriteBuffer,
		xWriteBufferLen,
		pcCommandString,
		string_buffer);

	/* Return if the user entered an invalid operation*/
	if (xReturn == pdFAIL)
	{
		/* Clear the return string */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid LCD Operation");
		return xReturn;
	}

	xQueueSend(q_lcd_req, string_buffer, portMAX_DELAY);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	memset(string_buffer, 0x00, LCD_STRING_MAX_LEN);

	xReturn = pdFAIL;
	return xReturn;
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
void task_lcd(void *param)
{
	// Initialize the display
	lcd_init_display();

	// Handled writing to the display
	char string_buffer[LCD_STRING_MAX_LEN];
	char lines[LCD_NUM_ROWS][LCD_NUM_COLS + 1];
	size_t line_lengths[LCD_NUM_ROWS];
	while (1)
	{
		/* Wait for a message */
		xQueueReceive(q_lcd_req, string_buffer, portMAX_DELAY);

		memset(lines, 0x00, sizeof(lines));
		memset(line_lengths, 0x00, sizeof(line_lengths));

		uint8_t row = 0u;
		uint8_t col = 0u;

		for (size_t i = 0; i < LCD_STRING_MAX_LEN && string_buffer[i] != '\0'; ++i)
		{
			char ch = string_buffer[i];

			if (ch == '\n')
			{
				if (row + 1u < LCD_NUM_ROWS)
				{
					row++;
					col = 0u;
				}
				else
				{
					break;
				}
				continue;
			}

			if (col >= LCD_NUM_COLS)
			{
				if (row + 1u < LCD_NUM_ROWS)
				{
					row++;
					col = 0u;
				}
				else
				{
					break;
				}
			}

			lines[row][col] = ch;
			col++;
			line_lengths[row] = col;
		}

		lcd_write_command(0x01u);

		for (uint8_t r = 0u; r < LCD_NUM_ROWS; ++r)
		{
			if (line_lengths[r] == 0u)
			{
				continue;
			}

			uint8_t start_col = (uint8_t)((LCD_NUM_COLS - line_lengths[r]) / 2u);
			lcd_set_cursor(start_col, r);

			for (size_t c = 0; c < line_lengths[r]; ++c)
			{
				lcd_write_data((uint8_t)lines[r][c]);
			}
		}
	}
}

/**
 * @brief
 * Initializes software resources related to the operation of
 * the LCD.  This function expects that the I2C bus had already
 * been initialized prior to the start of FreeRTOS.
 */
void task_lcd_init(void)
{
	/* Create the Queue used to control the LCD*/
	q_lcd_req = xQueueCreate(1, sizeof(char[LCD_STRING_MAX_LEN]));

	/* Register the CLI command */
	FreeRTOS_CLIRegisterCommand(&cmd_lcd);

	/* Create the task that will control the LCD */
	xTaskCreate(
		task_lcd,
		"Task LCD",
		configMINIMAL_STACK_SIZE * 2,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);

}
