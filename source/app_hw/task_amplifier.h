/******************************************************************************
* File Name: task_amplifier.h
*
* Description: This file contains the task and CLI command declarations for
*              amplifier control.
*
*******************************************************************************/

#ifndef TASK_AMPLIFIER_H_
#define TASK_AMPLIFIER_H_

/*******************************************************************************
* Header files
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define TASK_AMPLIFIER_PRIORITY     (2)
#define TASK_AMPLIFIER_STACK_SIZE   (1024)
#define AMPLIFIER_QUEUE_SIZE        (10)

/*******************************************************************************
* Message Types
*******************************************************************************/
typedef enum {
    AMP_MSG_PLAY_TONE,
    AMP_MSG_STOP_TONE
} amp_msg_type_t;

/*******************************************************************************
* Message Structure
*******************************************************************************/
typedef struct {
    amp_msg_type_t type;
} amp_msg_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern QueueHandle_t amplifier_queue;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void task_amplifier_init(void);
void task_amplifier(void *arg);
BaseType_t amplifier_send_command(amp_msg_t *msg);

#endif /* TASK_AMPLIFIER_H_ */