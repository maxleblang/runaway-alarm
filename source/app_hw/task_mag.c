/**
 * @file task_mag.c
 * @author
 * @brief Magnetometer (IIS2MDC) task and CLI interface using I2C
 * @version 0.2
 * @date 2025-11-02
 */

#include "task_mag.h"
#include "iis2mdc_reg.h"

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

static void task_mag(void *param);
static BaseType_t cli_handler_mag_read(char *writeBuffer,
                                       size_t writeBufferLen,
                                       const char *commandString);
static BaseType_t cli_handler_mag_whoami(char *writeBuffer,
                                         size_t writeBufferLen,
                                         const char *commandString);

static bool mag_configure(void);

/******************************************************************************/
/* Local Helper Prototypes                                                    */
/******************************************************************************/
static int32_t st_i2c_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len);
static int32_t st_i2c_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/

QueueHandle_t q_mag_req;
QueueHandle_t q_mag_rsp;

/* Magnetometer I2C address (7-bit) */
#define IIS2MDC_ADDR 0x1E

/* ST driver context */
static stmdev_ctx_t mag_ctx;

/******************************************************************************/
/* CLI Command Definitions                                                    */
/******************************************************************************/

static const CLI_Command_Definition_t cmd_mag_read = {
    "mag_read",
    "\r\nmag_read\r\n",
    cli_handler_mag_read,
    0};

static const CLI_Command_Definition_t cmd_mag_whoami = {
    "mag_who_am_i",
    "\r\nmag_who_am_i\r\n",
    cli_handler_mag_whoami,
    0};

/******************************************************************************/
/* ST Driver I2C Shims                                                        */
/******************************************************************************/

static int32_t st_i2c_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)handle;
    cy_rslt_t rslt;

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

    // Write subaddress (no stop)
    rslt = cyhal_i2c_master_write(&i2c_master_obj, IIS2MDC_ADDR, &reg, 1, 0, false);
    if (rslt == CY_RSLT_SUCCESS)
    {
        rslt = cyhal_i2c_master_read(&i2c_master_obj, IIS2MDC_ADDR, buf, len, 0, true);
    }

    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? 0 : -1;
}

static int32_t st_i2c_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)handle;
    cy_rslt_t rslt;
    uint8_t tmp[1 + 16]; // small writes only
    if (len > 16)
        return -1;

    tmp[0] = reg;
    memcpy(&tmp[1], buf, len);

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);
    rslt = cyhal_i2c_master_write(&i2c_master_obj, IIS2MDC_ADDR, tmp, len + 1, 0, true);
    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? 0 : -1;
}

/******************************************************************************/
/* Sensor Configuration                                                       */
/******************************************************************************/

static bool mag_configure(void)
{
    uint8_t who = 0;
    uint8_t rst = 0;

    mag_ctx.read_reg = st_i2c_read;
    mag_ctx.write_reg = st_i2c_write;
    mag_ctx.mdelay = NULL;
    mag_ctx.handle = NULL;

    /* Verify device ID */
    if (iis2mdc_device_id_get(&mag_ctx, &who) != 0 || who != IIS2MDC_ID)
    {
        task_print_error("IIS2MDC not found (WHO_AM_I=0x%02X)", who);
        return false;
    }

    /* Soft reset */
    iis2mdc_reset_set(&mag_ctx, PROPERTY_ENABLE);
    do
    {
        iis2mdc_reset_get(&mag_ctx, &rst);
    } while (rst);


    iis2mdc_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
    iis2mdc_data_rate_set(&mag_ctx, IIS2MDC_ODR_50Hz);
    iis2mdc_set_rst_mode_set(&mag_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);
    iis2mdc_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);
    iis2mdc_operating_mode_set(&mag_ctx, IIS2MDC_CONTINUOUS_MODE);

    task_print("IIS2MDC configured (50Hz continuous)\r\n");
    return true;
}

/******************************************************************************/
/* CLI Handlers                                                               */
/******************************************************************************/

static BaseType_t cli_handler_mag_read(char *writeBuffer,
                                       size_t writeBufferLen,
                                       const char *commandString)
{
    (void)writeBuffer;
    (void)writeBufferLen;
    (void)commandString;

    task_print("\n");

    mag_packet_t request;
    request.address = 0;
    request.return_queue = q_mag_rsp;

    if (xQueueSend(q_mag_req, &request, portMAX_DELAY) != pdPASS)
    {
        task_print_error("Failed to send mag_read request");
        return pdFALSE;
    }

    if (xQueueReceive(q_mag_rsp, &request, portMAX_DELAY) != pdPASS)
    {
        task_print_error("Failed to receive mag_read response");
        return pdFALSE;
    }

    task_print("Mag (mGauss): X=%.2f Y=%.2f Z=%.2f\r\n",
               request.mx, request.my, request.mz);

    return pdFALSE;
}

static BaseType_t cli_handler_mag_whoami(char *writeBuffer,
                                         size_t writeBufferLen,
                                         const char *commandString)
{
    (void)writeBuffer;
    (void)writeBufferLen;
    (void)commandString;

    task_print("\n");

    mag_packet_t request;
    request.address = IIS2MDC_WHO_AM_I;
    request.return_queue = q_mag_rsp;

    if (xQueueSend(q_mag_req, &request, portMAX_DELAY) != pdPASS)
    {
        task_print_error("Failed to send mag_who_am_i request");
        return pdFALSE;
    }

    if (xQueueReceive(q_mag_rsp, &request, portMAX_DELAY) != pdPASS)
    {
        task_print_error("Failed to receive mag_who_am_i response");
        return pdFALSE;
    }

    task_print("IIS2MDC WHO_AM_I = 0x%02X\r\n", request.value);
    return pdFALSE;
}

/******************************************************************************/
/* Magnetometer Task                                                          */
/******************************************************************************/

static void task_mag(void *param)
{
    (void)param;
    mag_packet_t request;

    while (1)
    {
        if (xQueueReceive(q_mag_req, &request, portMAX_DELAY) == pdPASS)
        {
            if (request.address == IIS2MDC_WHO_AM_I)
            {
                uint8_t val;
                iis2mdc_device_id_get(&mag_ctx, &val);
                request.value = val;
            }
            else
            {
                uint8_t drdy = 0;
                iis2mdc_mag_data_ready_get(&mag_ctx, &drdy);
                if (drdy)
                {
                    int16_t raw[3];
                    if (iis2mdc_magnetic_raw_get(&mag_ctx, raw) == 0)
                    {
                        request.mx = iis2mdc_from_lsb_to_mgauss(raw[0]);
                        request.my = iis2mdc_from_lsb_to_mgauss(raw[1]);
                        request.mz = iis2mdc_from_lsb_to_mgauss(raw[2]);
                    }
                    else
                    {
                        task_print_error("magnetic_raw_get failed");
                    }
                }
            }

            if (request.return_queue)
            {
                xQueueSend(request.return_queue, &request, portMAX_DELAY);
            }
        }
    }
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

void task_mag_init()
{
    /* Configure the magnetometer */
    if (!mag_configure())
    {
        task_print_error("Magnetometer configuration failed");
    }

    /* Create queues */
    q_mag_req = xQueueCreate(5, sizeof(mag_packet_t));
    q_mag_rsp = xQueueCreate(5, sizeof(mag_packet_t));

    /* Register CLI commands */
    FreeRTOS_CLIRegisterCommand(&cmd_mag_read);
    FreeRTOS_CLIRegisterCommand(&cmd_mag_whoami);

    /* Create magnetometer task */
    xTaskCreate(task_mag,
                "MAG Task",
                configMINIMAL_STACK_SIZE + 100,
                NULL,
                configMAX_PRIORITIES - 6,
                NULL);
}
