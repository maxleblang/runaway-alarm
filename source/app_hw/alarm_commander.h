/******************************************************************************
* File Name: alarm_commander.h
*
* Description: This file contains the task declarations for the alarm commander
*              which coordinates alarm events and responses.
*
*******************************************************************************/

#ifndef ALARM_COMMANDER_H_
#define ALARM_COMMANDER_H_

/*******************************************************************************
* Header files
*******************************************************************************/
#include "main.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define TASK_ALARM_COMMANDER_PRIORITY     (configMAX_PRIORITIES - 5)
#define TASK_ALARM_COMMANDER_STACK_SIZE   (configMINIMAL_STACK_SIZE)

/* Event bits for alarm commander */
#define ALARM_CMD_BIT_ALARM     (1 << 0)  /* Bit 0: Alarm triggered set by RTC*/
#define ALARM_CMD_BIT_OBJECT  (1 << 1)    /* Bit 1: Object detected set by TOF*/
#define ALARM_CMD_BIT_NOTIFY    (1 << 2)  /* Bit 2: Notification to check state*/

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern EventGroupHandle_t eg_alarm_commander;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void task_alarm_commander_init(void);

#endif /* ALARM_COMMANDER_H_ */
