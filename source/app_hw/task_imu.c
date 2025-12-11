/**
 * @file task_spi.c
 * @author
 * @brief Generic SPI task with CLI commands
 * @version 0.1
 * @date 2025-09-18
 *
 * @copyright Copyright (c) 2025
 */

#include "task_imu.h"
#include "pins.h"

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

static void task_imu(void *param);

static BaseType_t cli_handler_imu_read(char *writeBuffer, size_t writeBufferLen,
                                       const char *commandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queues used to send/receive SPI requests */
QueueHandle_t q_imu_req;
QueueHandle_t q_imu_rsp;

static const CLI_Command_Definition_t cmd_imu_read = {
    "imu_read",           /* command text */
    "\r\nimu_read\r\n",   /* command help text */
    cli_handler_imu_read, /* handler function */
    0                     /* requires 0 parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/*
 * cli_handler_imu_read
 * - Called when user types "imu_dump"
 * - Reads accel/gyro values and prints them to the console
 */
static BaseType_t cli_handler_imu_read(char *writeBuffer, size_t writeBufferLen,
                                       const char *commandString) {
  task_print("\n");

  imu_packet_t request;

  request.address = 0; // unused for IMU dump
  request.value = 0;
  request.return_queue = q_imu_rsp;

  /* Send request to SPI task */
  if (xQueueSend(q_imu_req, &request, portMAX_DELAY) != pdPASS) {
    task_print_error("Failed to send imu_read request");
    return pdFALSE;
  }

  /* Wait for response */
  if (xQueueReceive(q_imu_rsp, &request, portMAX_DELAY) != pdPASS) {
    task_print_error("Failed to receive imu_read response");
    return pdFALSE;
  }

  task_print("Accel raw: X=%d Y=%d Z=%d | Gyro raw: X=%d Y=%d Z=%d\r\n",
             request.ax, request.ay, request.az, request.gx, request.gy,
             request.gz);

  return pdFALSE;
}

static void task_imu(void *param) {
   
  imu_init(&mSPI, IMU_CS_N);
  imu_packet_t request;


  while (1) {
    if (xQueueReceive(q_imu_req, &request, portMAX_DELAY) == pdPASS) {
      xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);

      if (request.address) {
        request.value = imu_read_reg(request.address);
      } else {
        imu_read_accel_gyro(&request.ax, &request.ay, &request.az, &request.gx,
                            &request.gy, &request.gz);
      }

      xSemaphoreGive(Semaphore_SPI);

      if (request.return_queue) {
        xQueueSend(request.return_queue, &request, portMAX_DELAY);
      }
    }
  }
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/*
 * task_spi_init
 * - Initializes the SPI interface,
 *   creates queues, registers CLI commands,
 *   and creates the SPI task.
 */
void task_imu_init() {
  /* Initialize CS pin */
  cyhal_gpio_init(IMU_CS_N, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);

  /* Create queues */
  q_imu_req = xQueueCreate(5, sizeof(imu_packet_t));
  q_imu_rsp = xQueueCreate(5, sizeof(imu_packet_t));

  /* Register CLI commands */

  FreeRTOS_CLIRegisterCommand(&cmd_imu_read);

  /* Create SPI task */
  xTaskCreate(task_imu, "IMU Task", configMINIMAL_STACK_SIZE, NULL,
              configMAX_PRIORITIES - 6, NULL);
}