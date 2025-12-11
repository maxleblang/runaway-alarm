/*
 * opt3001.h
 *
 *  Created on: Oct 20, 2020
 *      Author: Joe Krachey
 */

#ifndef __TASK_RTC_H__
#define __TASK_RTC_H__

#include "main.h"
#include "task_console.h"
#include "i2c.h"
#include <stdint.h>

#define BQ32002DR_SUBORDINATE_ADDR                  0x68

#define ALARM_CHECK_INTERVAL_MS                     5000 // Check alarm every 5 seconds

extern char alarm_time_str[8]; // Alarm time in HH:MM string format
extern SemaphoreHandle_t alarm_time_mutex; // Mutex to protect alarm_time_str
extern QueueHandle_t amplifier_queue; // Queue to communicate with amplifier task
extern QueueHandle_t q_lcd_req; // Queue to communicate with lcd task

typedef enum 
{
    RTC_OP_SET_TIME,
    RTC_OP_READ_TIME,
    RTC_OP_SET_ALARM,
    RTC_OP_INVALID,
}rtc_operation_t;

// TODO: Adjust register addresses for BQ32002DR RTC
typedef enum 
{
    RTC_REG_ADDR_SECONDS          = 0x00,
    RTC_REG_ADDR_MINUTES          = 0x01,
    RTC_REG_ADDR_HOURS            = 0x02,
}rtc_reg_addr_t;


typedef struct 
{
    uint8_t seconds; // BCD format
    uint8_t minutes; // BCD format
    uint8_t hours;   // BCD format
    uint8_t day;     // BCD format
    uint8_t date;    // BCD format
    uint8_t month;   // BCD format
    uint8_t year;    // BCD format
} rtc_time_date_t;

typedef struct 
{
    rtc_operation_t operation;
    rtc_time_date_t time_date;
    QueueHandle_t return_queue;
} rtc_packet_t;

extern QueueHandle_t q_rtc_req;
extern QueueHandle_t q_rtc_rsp;

void task_rtc_init(void);
uint8_t decimal_to_bcd(uint8_t decimal);

#endif
