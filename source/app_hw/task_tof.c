/**
 * @file task_tof.c
 * @author Aayushi Singh
 * @brief FreeRTOS task for TOF (Time-of-Flight) sensor management
 * @version 0.1
 * @date 2025-12-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "task_tof.h"
#include "VL53L3CX_ULP_api.h"
#include "alarm_commander.h"
#include "pins.h"
#include "cyhal.h"
#include <stdio.h>

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/

#define TOF_DEV_ADDRESS 0x29
#define TOF_THRESHOLD_MM 600

static cyhal_gpio_callback_data_t tof_callback_data;
static TaskHandle_t tof_task_handle = NULL;
static bool prev_object_detected = false;

/*****************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

static void task_tof(void *param);
static void tof_gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/******************************************************************************/
/* Interrupt Handler                                                          */
/******************************************************************************/

/**
 * @brief GPIO interrupt handler for TOF sensor
 * 
 * Triggers on both edges - rising and falling
 */
static void tof_gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event) {
  (void)handler_arg;
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  /* Read GPIO state immediately in ISR to capture the edge */
  bool current_state = cyhal_gpio_read(TOF_GPIO);
  
  /* Only notify task on falling edge (high to low) */
  if (!current_state && (event & CYHAL_GPIO_IRQ_FALL)) {
    /* Wake up the task to handle the detection */
    if (tof_task_handle != NULL) {
      vTaskNotifyGiveFromISR(tof_task_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/******************************************************************************/
/* Task Implementation                                                        */
/******************************************************************************/

/**
 * @brief Main TOF task - interrupt-driven object detection
 *
 * This task:
 * - Initializes the TOF sensor
 * - Configures interrupt for object detection below 600mm
 * - Registers GPIO interrupt handler for falling edge
 * - Starts continuous ranging
 * - Waits for interrupt notifications from ISR
 */
static void task_tof(void *param) {
  (void)param;
  
  uint8_t result;
  cy_rslt_t cy_result;
  uint16_t sensor_id;
  
  /* Save task handle for ISR notification */
  tof_task_handle = xTaskGetCurrentTaskHandle();

  /* Initialize XSHUT pin - must be HIGH to enable sensor */
  cy_result = cyhal_gpio_init(TOF_XSHUT, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
  if (cy_result != CY_RSLT_SUCCESS) {
    task_print_error("TOF: XSHUT init failed\r\n");
    vTaskDelete(NULL);
    return;
  }
  
  vTaskDelay(pdMS_TO_TICKS(10)); /* Give sensor time to power up */

  /* Initialize GPIO as input for interrupt detection */
  cy_result = cyhal_gpio_init(TOF_GPIO, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
  if (cy_result != CY_RSLT_SUCCESS) {
    task_print_error("TOF: GPIO init failed\r\n");
    vTaskDelete(NULL);
    return;
  }

  /* Check if sensor is present */
  result = VL53L3CX_ULP_GetSensorId(TOF_DEV_ADDRESS, &sensor_id);
  if (result || (sensor_id != 0xEAAA)) {
    task_print_error("TOF: Sensor not detected at address 0x%02X\r\n", TOF_DEV_ADDRESS);
    vTaskDelete(NULL);
    return;
  }

  /* Initialize VL53L3CX sensor */
  result = VL53L3CX_ULP_SensorInit(TOF_DEV_ADDRESS);
  if (result) {
    task_print_error("TOF: Sensor initialization failed\r\n");
    vTaskDelete(NULL);
    return;
  }

  task_print("TOF: Sensor ready (ID: 0x%04X)\r\n", sensor_id);

  /* Speed up inter-measurement period from 1000ms to 100ms (10x faster) */
  /* Note: Min is 20ms per datasheet, but needs to be > macro timing */
  result = VL53L3CX_ULP_SetInterMeasurementInMs(TOF_DEV_ADDRESS, 50);
  if (result) {
    task_print_error("TOF: Set inter-measurement failed (continuing with default)\r\n");
    /* Don't fail - just continue with default timing */
  }

  /* Configure interrupt to trigger ONLY below 600mm */
  result = VL53L3CX_ULP_SetInterruptConfiguration(TOF_DEV_ADDRESS, TOF_THRESHOLD_MM, 1);
  if (result) {
    task_print_error("TOF: Interrupt configuration failed\r\n");
    vTaskDelete(NULL);
    return;
  }

  /* Start continuous ranging */
  result = VL53L3CX_ULP_StartRanging(TOF_DEV_ADDRESS);
  if (result) {
    task_print_error("TOF: Start ranging failed\r\n");
    vTaskDelete(NULL);
    return;
  }

  task_print("TOF: Ranging started. Monitoring for objects below %dmm...\r\n", TOF_THRESHOLD_MM);

  /* Register interrupt handler for both edges */
  tof_callback_data.callback = tof_gpio_interrupt_handler;
  tof_callback_data.callback_arg = NULL;
  cyhal_gpio_register_callback(TOF_GPIO, &tof_callback_data);
  cyhal_gpio_enable_event(TOF_GPIO, CYHAL_GPIO_IRQ_BOTH, 3, true);
  
  task_print("TOF: Interrupt-driven mode enabled (both edges)\r\n");

  /* Wait for interrupt notifications - task sleeps until ISR wakes it up */
  while (1) {
    /* Block indefinitely waiting for notification from ISR (only sent on falling edge) */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    /* ISR detected falling edge - clear the sensor interrupt via I2C */
    VL53L3CX_ULP_ClearInterrupt(TOF_DEV_ADDRESS);
    
    /* Only set event bits on rising edge (transition from no object to object detected) */
    if (!prev_object_detected) {
      prev_object_detected = true;
      xEventGroupSetBits(eg_alarm_commander, ALARM_CMD_BIT_OBJECT | ALARM_CMD_BIT_NOTIFY);
    }
  }
}

/**
 * @brief Reset the object detection state
 * 
 * Called by alarm_commander after clearing the OBJECT bit to allow
 * detection of the next object (simulates falling edge)
 */
void task_tof_reset_object_state(void) {
  prev_object_detected = false;
}

/******************************************************************************/
/* Public Function Implementation                                             */
/******************************************************************************/

/**
 * @brief Initialize the TOF task
 *
 * This function creates the TOF FreeRTOS task
 */
void task_tof_init(void) {
  /* Create the TOF task */
  xTaskCreate(task_tof, "Task_TOF", configMINIMAL_STACK_SIZE + 256, NULL,
              configMAX_PRIORITIES - 5, NULL);
}
