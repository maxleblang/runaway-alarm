/******************************************************************************
* File Name: task_amplifier.c
*
* Description: This file contains the FreeRTOS task and CLI commands for
*              amplifier and speaker control.
*
*******************************************************************************/

/*******************************************************************************
* Header files
*******************************************************************************/
#include "task_amplifier.h"
#include "amplifier.h"
#include "FreeRTOS_CLI.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
* Global Variables
*******************************************************************************/
QueueHandle_t amplifier_queue;
static TaskHandle_t amplifier_task_handle = NULL;

/*******************************************************************************
* CLI Function Prototypes
*******************************************************************************/
static BaseType_t cli_amp_play(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t cli_amp_stop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t cli_amp_status(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/*******************************************************************************
* CLI Command Definitions
*******************************************************************************/
static const CLI_Command_Definition_t amp_play_command = {
    "amp_play",
    "amp_play\r\n    Play sine wave tone\r\n",
    cli_amp_play,
    0
};

static const CLI_Command_Definition_t amp_stop_command = {
    "amp_stop",
    "amp_stop\r\n    Stop playing tone\r\n",
    cli_amp_stop,
    0
};

static const CLI_Command_Definition_t amp_status_command = {
    "amp_status",
    "amp_status\r\n    Show amplifier status\r\n",
    cli_amp_status,
    0
};

/*******************************************************************************
* Function Name: task_amplifier_init
********************************************************************************
* Summary:
*  Initializes the amplifier task, queue, and CLI commands.
*******************************************************************************/
void task_amplifier_init(void)
{
    /* Create message queue */
    amplifier_queue = xQueueCreate(AMPLIFIER_QUEUE_SIZE, sizeof(amp_msg_t));
    if (!amplifier_queue)
    {
        return;
    }
    
    /* Initialize amplifier hardware */
    if (amplifier_init() != CY_RSLT_SUCCESS)
    {
        return;
    }
    
    /* Register CLI commands */
    FreeRTOS_CLIRegisterCommand(&amp_play_command);
    FreeRTOS_CLIRegisterCommand(&amp_stop_command);
    FreeRTOS_CLIRegisterCommand(&amp_status_command);
    
    /* Create amplifier task */
    xTaskCreate(task_amplifier, "Task Amp", TASK_AMPLIFIER_STACK_SIZE,
                NULL, TASK_AMPLIFIER_PRIORITY, &amplifier_task_handle);
}

/*******************************************************************************
* Function Name: task_amplifier
********************************************************************************
* Summary:
*  Main amplifier task that processes messages from the queue.
*
* Parameters:
*  arg - Unused parameter
*******************************************************************************/
void task_amplifier(void *arg)
{
    (void)arg;
    amp_msg_t received_msg;
    
    while (1)
    {
        if (xQueueReceive(amplifier_queue, &received_msg, portMAX_DELAY) == pdTRUE)
        {
            switch (received_msg.type)
            {
                case AMP_MSG_PLAY_TONE:
                    amplifier_play_tone(0, WAVE_SINE);
                    break;
                    
                case AMP_MSG_STOP_TONE:
                    amplifier_stop_tone();
                    break;
                    
                default:
                    break;
            }
        }
    }
}

/*******************************************************************************
* Function Name: amplifier_send_command
********************************************************************************
* Summary:
*  Sends a command message to the amplifier task queue.
*
* Parameters:
*  msg - Pointer to message structure
*
* Return:
*  BaseType_t - pdTRUE if successful, pdFAIL otherwise
*******************************************************************************/
BaseType_t amplifier_send_command(amp_msg_t *msg)
{
    if (!amplifier_queue)
    {
        return pdFAIL;
    }
    return xQueueSend(amplifier_queue, msg, pdMS_TO_TICKS(100));
}

/*******************************************************************************
* CLI Command Implementations
*******************************************************************************/

static BaseType_t cli_amp_play(char *pcWriteBuffer, size_t xWriteBufferLen,
                              const char *pcCommandString)
{
    (void)pcCommandString;
    
    amp_msg_t msg = {.type = AMP_MSG_PLAY_TONE};
    
    if (amplifier_send_command(&msg) == pdTRUE)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, 
                "Playing sine wave on P9_6\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR: Failed to send command\r\n");
    }
    
    return pdFALSE;
}

static BaseType_t cli_amp_stop(char *pcWriteBuffer, size_t xWriteBufferLen,
                              const char *pcCommandString)
{
    (void)pcCommandString;
    
    amp_msg_t msg = {.type = AMP_MSG_STOP_TONE};
    
    if (amplifier_send_command(&msg) == pdTRUE)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Tone stopped\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR: Failed to send command\r\n");
    }
    
    return pdFALSE;
}

static BaseType_t cli_amp_status(char *pcWriteBuffer, size_t xWriteBufferLen,
                                const char *pcCommandString)
{
    (void)pcCommandString;
    
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "Amplifier Status:\r\n"
             "  Playing: %s\r\n"
             "  Waveform: Sine\r\n"
             "  DAC Pin: P9_6\r\n",
             amplifier_is_playing() ? "YES" : "NO");
    
    return pdFALSE;
}