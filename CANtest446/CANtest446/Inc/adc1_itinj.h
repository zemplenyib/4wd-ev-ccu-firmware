/*******************************************************************************
 * File Name:	adc1_itinj.h
 * Description:	Driver header for handling multiple channels on ADC1 by DMA
 *******************************************************************************
 *
 * Copyright(c) 2018 MTA SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file adc1_itinj.h
 * @author Alexandros Soumelidis
 * @date 5 Apr 2018
 * @brief Driver header for handling multiple channels on ADC1 by DMA.
 *
 * Settings: Scanned Injected Conversion on multiple analog channels
 * Trigger:  Timer 1 Update Event
 *                
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"

/* Constants & Macros --------------------------------------------------------*/

#define	POT1_CH			1				// Channel Rank of Potentiometer 1
#define	POT2_CH			2				// Channel Rank of Potentiometer 2

/* Global function prototypes ------------------------------------------------*/

/**
  * @brief  Initializing / starting ADC Channels
  * @param  none
  * @retval none
  */

void InitADCmodule (void);

/**
  * @brief  Reading measurement value associated with the ADC Channel.
  *
  *	Checking whether the measurement is ready, and in this case
  *	the measured value is returned. Overrun is also detected.
  *
  * @param  ch - channel index
  * @param  value [out by ref] - measurement value if ready
  * @retval 1 if measurement is ready, 0 not ready, -1 overrun error
  * @note	In case of overrun: the last value is given and error is cleared
  */

int GetADCChannelValue (uint8_t ch, uint16_t *value);

/************************ (C) COPYRIGHT MTA SZTAKI *****END OF FILE************/
