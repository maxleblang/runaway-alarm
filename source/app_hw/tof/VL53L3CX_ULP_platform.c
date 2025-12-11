/**
 * @file VL53L3CX_ULP_platform.c
 * @brief Platform I2C layer for VL53L3CX ULP driver (PSoC6)
 */

#include "VL53L3CX_ULP_platform.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cyhal.h"

/***********************************************************************
 * Internal helper: Write 16-bit register index (MSB first)
 **********************************************************************/
static inline cy_rslt_t write_reg_addr(uint16_t dev, uint16_t reg)
{
    uint8_t buf[2];
    buf[0] = (reg >> 8) & 0xFF;   // MSB
    buf[1] = reg & 0xFF;          // LSB

    return cyhal_i2c_master_write(
        &i2c_master_obj,
        dev,
        buf,
        2,
        0,
        false      // DO NOT send STOP (we need repeated start)
    );
}

/***********************************************************************
 * READ BYTE
 **********************************************************************/
uint8_t VL53L3CX_ULP_RdByte(uint16_t dev, uint16_t reg, uint8_t *value)
{
    cy_rslt_t rslt;

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    rslt = write_reg_addr(dev, reg);
    if (rslt != CY_RSLT_SUCCESS) {
        xSemaphoreGive(Semaphore_I2C);
        return 255;
    }

    rslt = cyhal_i2c_master_read(
        &i2c_master_obj,
        dev,
        value,
        1,
        0,
        true     // STOP
    );

    xSemaphoreGive(Semaphore_I2C);
    return (rslt == CY_RSLT_SUCCESS) ? 0 : 255;
}

/***********************************************************************
 * READ WORD (16-bit, MSB first)
 **********************************************************************/
uint8_t VL53L3CX_ULP_RdWord(uint16_t dev, uint16_t reg, uint16_t *value)
{
    cy_rslt_t rslt;
    uint8_t buf[2];

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    rslt = write_reg_addr(dev, reg);
    if (rslt != CY_RSLT_SUCCESS) {
        xSemaphoreGive(Semaphore_I2C);
        return 255;
    }

    rslt = cyhal_i2c_master_read(
        &i2c_master_obj,
        dev,
        buf,
        2,
        0,
        true
    );

    xSemaphoreGive(Semaphore_I2C);

    if (rslt != CY_RSLT_SUCCESS)
        return 255;

    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/***********************************************************************
 * READ DWORD (32-bit, MSB first)
 **********************************************************************/
uint8_t VL53L3CX_ULP_RdDWord(uint16_t dev, uint16_t reg, uint32_t *value)
{
    cy_rslt_t rslt;
    uint8_t buf[4];

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    rslt = write_reg_addr(dev, reg);
    if (rslt != CY_RSLT_SUCCESS) {
        xSemaphoreGive(Semaphore_I2C);
        return 255;
    }

    rslt = cyhal_i2c_master_read(
        &i2c_master_obj,
        dev,
        buf,
        4,
        0,
        true
    );

    xSemaphoreGive(Semaphore_I2C);

    if (rslt != CY_RSLT_SUCCESS)
        return 255;

    *value = ((uint32_t)buf[0] << 24) |
             ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8) |
             (uint32_t)buf[3];

    return 0;
}

/***********************************************************************
 * WRITE BYTE
 **********************************************************************/
uint8_t VL53L3CX_ULP_WrByte(uint16_t dev, uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = value;

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    cy_rslt_t rslt = cyhal_i2c_master_write(
        &i2c_master_obj,
        dev,
        buf,
        3,
        0,
        true
    );

    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? 0 : 255;
}

/***********************************************************************
 * WRITE WORD (MSB first)
 **********************************************************************/
uint8_t VL53L3CX_ULP_WrWord(uint16_t dev, uint16_t reg, uint16_t value)
{
    uint8_t buf[4];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    cy_rslt_t rslt = cyhal_i2c_master_write(
        &i2c_master_obj,
        dev,
        buf,
        4,
        0,
        true
    );

    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? 0 : 255;
}

/***********************************************************************
 * WRITE DWORD (MSB first)
 **********************************************************************/
uint8_t VL53L3CX_ULP_WrDWord(uint16_t dev, uint16_t reg, uint32_t value)
{
    uint8_t buf[6];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = (value >> 24) & 0xFF;
    buf[3] = (value >> 16) & 0xFF;
    buf[4] = (value >> 8) & 0xFF;
    buf[5] = value & 0xFF;

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    cy_rslt_t rslt = cyhal_i2c_master_write(
        &i2c_master_obj,
        dev,
        buf,
        6,
        0,
        true
    );

    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? 0 : 255;
}

/***********************************************************************
 * Wait function
 **********************************************************************/
void VL53L3CX_ULP_WaitMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
