/*
 * ece453.c
 *
 *  Created on: Jan 18, 2022
 *      Author: Joe Krachey
 */

#include "spi.h"
#include "cy_result.h"
#include "cyhal_psoc6_01_116_bga_ble.h"
#include "pins.h"
#include "main.h"


cyhal_spi_t mSPI;
SemaphoreHandle_t Semaphore_SPI;

/* This function initializes the SPI interface for the specified module site */
/* It does NOT configure the Chip Select (CS) pin.  The CS pins should be */
/* configured separately a GPIO pin. */
cy_rslt_t spi_init()
{
	cy_rslt_t   rslt;
	cyhal_gpio_t mosi = IMU_MOSI;
	cyhal_gpio_t miso = IMU_MISO;
	cyhal_gpio_t sclk = IMU_SCLK;
	cyhal_gpio_t cs_n = IMU_CS_N;

	memset(&mSPI, 0, sizeof(mSPI));

    // Configuring the  SPI master:  Specify the SPI interface pins, frame size, SPI Motorola mode
    // and master mode
    rslt = cyhal_spi_init(
    						&mSPI,
							mosi,			// MOSI Pin
							miso,			// MISO Pin
							sclk,			// Clock Pin
							cs_n, 	// CS -- Will control using an IO pin
							NULL,						// Clock Source -- if not provided a new clock will be allocated
							8,							// Bits per frame
							CYHAL_SPI_MODE_00_MSB,		// SPI Mode
							false						// Is Subordinate??
						);

    if (rslt == CY_RSLT_SUCCESS)
    {
        // Set the data rate
    	rslt = cyhal_spi_set_frequency(&mSPI, SPI_FREQ);

		if (rslt != CY_RSLT_SUCCESS)
		{
			return rslt;
		}
    }

	/* Create the semaphore used to grant mutual exclusion while using the SPI bus */
	Semaphore_SPI = xSemaphoreCreateBinary();

	xSemaphoreGive(Semaphore_SPI);

	return CY_RSLT_SUCCESS;

}


