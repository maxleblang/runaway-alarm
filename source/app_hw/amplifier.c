/******************************************************************************
* File Name: amplifier.c
*
* Description: Simple CTDAC sine wave generation using DMA on pin P9_6
*
*******************************************************************************/

/*******************************************************************************
* Header files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdbool.h>
#include "cycfg_peripherals.h"
#include "cycfg_dmas.h"

/* Waveform types */
typedef enum
{
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH
} waveform_type_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
static bool is_playing = false;

/* Sine wave lookup table from working example - exactly 101 samples */
static uint32_t sineWaveLUT[] = { 
    0x7FF, 0x880, 0x900, 0x97F, 0x9FC, 0xA78, 0xAF1,
    0xB67, 0xBD9, 0xC48, 0xCB2, 0xD18, 0xD79, 0xDD4, 0xE29, 0xE77, 0xEC0,
    0xF01, 0xF3C, 0xF6F, 0xF9A, 0xFBE, 0xFDA, 0xFEE, 0xFFA, 0xFFF, 0xFFA,
    0xFEE, 0xFDA, 0xFBE, 0xF9A, 0xF6F, 0xF3C, 0xF01, 0xEC0, 0xE77, 0xE29,
    0xDD4, 0xD79, 0xD18, 0xCB2, 0xC48, 0xBD9, 0xB67, 0xAF1, 0xA78, 0x9FC,
    0x97F, 0x900, 0x880, 0x7FF, 0x77E, 0x6FE, 0x67F, 0x602, 0x586, 0x50D,
    0x497, 0x425, 0x3B6, 0x34C, 0x2E6, 0x285, 0x22A, 0x1D5, 0x187, 0x13E,
    0x0FD, 0x0C2, 0x08F, 0x064, 0x040, 0x024, 0x010, 0x004, 0x000, 0x004,
    0x010, 0x024, 0x040, 0x064, 0x08F, 0x0C2, 0x0FD, 0x13E, 0x187, 0x1D5,
    0x22A, 0x285, 0x2E6, 0x34C, 0x3B6, 0x425, 0x497, 0x50D, 0x586, 0x602,
    0x67F, 0x6FE, 0x77E 
};

/* External DMA configuration from Device Configurator */
extern const cy_stc_dma_descriptor_config_t DMA_Descriptor_0_config;
extern cy_stc_dma_descriptor_t DMA_Descriptor_0;
extern const cy_stc_dma_channel_config_t DMA_channelConfig;

/* External VDAC configuration from Device Configurator */
extern const cy_stc_ctdac_config_t VDAC_config;

/*******************************************************************************
* Function Name: vdac_start
********************************************************************************
* Summary:
*  Initializes and enables the CTDAC. Matches the example code exactly.
*******************************************************************************/
static void vdac_start(void)
{
    cy_en_ctdac_status_t status;
    status = Cy_CTDAC_Init(VDAC_HW, &VDAC_config);
    if (CY_CTDAC_SUCCESS == status)
    {
        /* Turn on the hardware block */
        Cy_CTDAC_Enable(VDAC_HW);
    }
    Cy_CTDAC_SetValue(VDAC_HW, 0x00);
}

/*******************************************************************************
* Function Name: dma_start
********************************************************************************
* Summary:
*  Initializes and starts DMA. Matches the example code exactly.
*
* Parameters:
*  lut - Pointer to lookup table array
*  dest - Pointer to destination address
*******************************************************************************/
static void dma_start(uint32_t lut[], uint32_t *dest)
{
    cy_en_dma_status_t dma_init_status;

    /* Initialize descriptor 0 */
    dma_init_status = Cy_DMA_Descriptor_Init(&DMA_Descriptor_0,
            &DMA_Descriptor_0_config);
    if (CY_DMA_SUCCESS != dma_init_status)
    {
        CY_ASSERT(0);
    }
    dma_init_status = Cy_DMA_Channel_Init(DMA_HW, DMA_CHANNEL, &DMA_channelConfig);

    if (CY_DMA_SUCCESS != dma_init_status)
    {
        CY_ASSERT(0);
    }

    /* Set source address as the LUT for descriptor 0 */
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_Descriptor_0, (uint32_t*)lut);
    /* Set destination address as the CTDAC buffer register */
    Cy_DMA_Descriptor_SetDstAddress(&DMA_Descriptor_0, (uint32_t*)dest);

    /* Enable the descriptor */
    Cy_DMA_Channel_Enable(DMA_HW, DMA_CHANNEL);
    Cy_DMA_Enable(DMA_HW);
}

/*******************************************************************************
* Function Name: amplifier_init
********************************************************************************
* Summary:
*  Initializes the VDAC hardware.
*
* Return:
*  cy_rslt_t - Result of the initialization
*******************************************************************************/
cy_rslt_t amplifier_init(void)
{
    vdac_start();
    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: amplifier_play_tone
********************************************************************************
* Summary:
*  Starts playing a sine wave tone.
*
* Parameters:
*  frequency - Not used (frequency set by DMA trigger rate)
*  waveform - Not used (sine wave only for now)
*
* Return:
*  cy_rslt_t - Result of the operation
*******************************************************************************/
cy_rslt_t amplifier_play_tone(uint16_t frequency, waveform_type_t waveform)
{
    (void)frequency;
    (void)waveform;
    
    if (!is_playing)
    {
        dma_start(sineWaveLUT, (uint32_t*)&(VDAC_HW->CTDAC_VAL_NXT));
        is_playing = true;
    }
    
    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: amplifier_stop_tone
********************************************************************************
* Summary:
*  Stops playing the tone.
*
* Return:
*  cy_rslt_t - Result of the operation
*******************************************************************************/
cy_rslt_t amplifier_stop_tone(void)
{
    if (is_playing)
    {
        /* Disable DMA */
        Cy_DMA_Channel_Disable(DMA_HW, DMA_CHANNEL);
        Cy_DMA_Disable(DMA_HW);
        
        /* Set DAC to zero */
        Cy_CTDAC_SetValue(VDAC_HW, 0x00);
        
        is_playing = false;
    }
    
    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: amplifier_is_playing
********************************************************************************
* Summary:
*  Checks if a tone is currently playing.
*
* Return:
*  bool - true if playing, false otherwise
*******************************************************************************/
bool amplifier_is_playing(void)
{
    return is_playing;
}