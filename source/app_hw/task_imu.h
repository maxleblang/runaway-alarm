#ifndef __TASK_SPI_H__
#define __TASK_SPI_H__

#include "main.h"
#include "spi.h"
#include "imu.h"
#include "task_console.h" 


typedef struct {
    uint8_t address;
    uint8_t value;
    int32_t ax, ay, az; // accelerometer data
    int32_t gx, gy, gz; // gyroscope data
    QueueHandle_t return_queue;
} imu_packet_t;

extern QueueHandle_t q_imu_req, q_imu_rsp;

extern SemaphoreHandle_t Semaphore_SPI;
void task_imu_init();

#endif
