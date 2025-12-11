/******************************************************************************
* File Name: amplifier.h
*
* Description: Simple CTDAC sine wave generation on pin P9_6
*
*******************************************************************************/

#ifndef AMPLIFIER_H_
#define AMPLIFIER_H_

/*******************************************************************************
* Header files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include <stdbool.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define AMPLIFIER_DAC_PIN       P9_6

/* Waveform types (for compatibility) */
typedef enum
{
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH
} waveform_type_t;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_rslt_t amplifier_init(void);
cy_rslt_t amplifier_play_tone(uint16_t frequency, waveform_type_t waveform);
cy_rslt_t amplifier_stop_tone(void);
bool amplifier_is_playing(void);

/* Stub functions for task compatibility */
cy_rslt_t amplifier_set_volume(uint8_t volume);
uint8_t amplifier_get_volume(void);
cy_rslt_t amplifier_mute(void);
cy_rslt_t amplifier_unmute(void);
bool amplifier_is_muted(void);
cy_rslt_t amplifier_enable(void);
cy_rslt_t amplifier_disable(void);
bool amplifier_is_enabled(void);
uint16_t amplifier_get_frequency(void);
waveform_type_t amplifier_get_waveform(void);

#endif /* AMPLIFIER_H_ */