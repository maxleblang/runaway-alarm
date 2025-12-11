/*
 * opt3001.h
 *
 *  Created on: Oct 20, 2020
 *      Author: Joe Krachey
 */

#ifndef __TASK_LCD_H__
#define __TASK_LCD_H__

#include "main.h"
#include "task_console.h"
#include "i2c.h"

#define CN0295D_SUBORDINATE_ADDR                 0x27

#define LCD_STRING_MAX_LEN                       32

#define LCD_CTRL_RS (0x01u)
#define LCD_CTRL_RW (0x02u)
#define LCD_CTRL_EN (0x04u)
#define LCD_CTRL_BL (0x08u)
#define LCD_NUM_COLS (16u)
#define LCD_NUM_ROWS (2u)

extern QueueHandle_t q_lcd_req;

void task_lcd_init(void);

#endif
