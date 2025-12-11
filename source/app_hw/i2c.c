/**
 * @file i2c.c
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2025-08-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "i2c.h"
#include "pins.h"

cyhal_i2c_t i2c_master_obj;
SemaphoreHandle_t Semaphore_I2C;

// Define the I2C master configuration structure
cyhal_i2c_cfg_t i2c_master_config =
	{
		CYHAL_I2C_MODE_MASTER,
		0, // address is not used for master mode
		I2C_MASTER_FREQUENCY};

/** Initialize the I2C bus to the specified module site
 *
 * @param - None
 */
cy_rslt_t i2c_init()
{
	cyhal_gpio_t sda = I2C_SDA;
	cyhal_gpio_t scl = I2C_SCL;

	cy_rslt_t rslt;

	// Initialize I2C master, set the SDA and SCL pins and assign a new clock
	rslt = cyhal_i2c_init(&i2c_master_obj, sda, scl, NULL);

	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	// Configure the I2C resource to be master
	rslt = cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config);

	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	/* Create the binary semaphore that will ensure mutual exclusion while using
	   the I2C bus */
	Semaphore_I2C = xSemaphoreCreateBinary();

	/* Need to give the semaphore so that the first task that attempts to take
	   the semaphore is successful */
	xSemaphoreGive(Semaphore_I2C);

	return CY_RSLT_SUCCESS;
}
