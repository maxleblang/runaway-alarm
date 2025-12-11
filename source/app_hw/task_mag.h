/**
 * @file task_mag.h
 * @author
 * @brief Header for Magnetometer (IIS2MDC) FreeRTOS task + CLI interface
 * @version 0.1
 * @date 2025-10-29
 */

#ifndef TASK_MAG_H
#define TASK_MAG_H

#include "iis2mdc_reg.h"
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "task_console.h"

/******************************************************************************/
/* Data Structures                                                            */
/******************************************************************************/

/**
 * @brief Packet structure for magnetometer task request/response.
 *
 * Used to pass requests from CLI to the magnetometer task
 * and return results via FreeRTOS queues.
 */
typedef struct
{
    uint8_t address;              /**< Register address (e.g., WHO_AM_I) */
    uint8_t value;                /**< Register value for WHO_AM_I */
    float mx;                     /**< X magnetic field (mGauss) */
    float my;                     /**< Y magnetic field (mGauss) */
    float mz;                     /**< Z magnetic field (mGauss) */
    QueueHandle_t return_queue;   /**< Queue to send response to */
} mag_packet_t;

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/

extern QueueHandle_t q_mag_req;   /**< Request queue handle */
extern QueueHandle_t q_mag_rsp;   /**< Response queue handle */

/******************************************************************************/
/* Public Function Prototypes                                                 */
/******************************************************************************/

/**
 * @brief Initialize the IIS2MDC magnetometer task and CLI interface.
 *
 * - Initializes I2C and the IIS2MDC magnetometer
 * - Creates FreeRTOS request/response queues
 * - Registers CLI commands ("mag_read", "mag_who_am_i")
 * - Spawns the magnetometer background task
 *
 * @param module_site  Board-specific I2C module identifier
 */
void task_mag_init();

#endif /* TASK_MAG_H */
