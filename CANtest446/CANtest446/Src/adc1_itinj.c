/*******************************************************************************
 * File Name:	adc1_itinj.c
 * Description:	 Driver program for handling multiple channels on ADC1 by IT.
 *******************************************************************************
 *
 * Copyright(c) 2018 MTA SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file adc1_itinj.c
 * @author Alexandros Soumelidis
 * @date 5 Apr 2018
 * @brief Driver program for handling multiple channels on ADC1 by IT.
 *
 * Settings: Scanned Injected Conversion on multiple analog channels
 * Trigger:  Timer 1 Update Event
 *                
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc1_itinj.h"

/* Constants & Macros --------------------------------------------------------*/

#define	POT1_AVN		64				// Averaging Number for Potentiometer 1
#define	POT2_AVN		64				// Averaging Number for Potentiometer 2

/* External variables --------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;

/* Private variables ---------------------------------------------------------*/

volatile uint16_t pot1ADCval = 0;		// Potentiometer 1 value
volatile uint32_t pot1ADCacc = 0;		// Potentiometer 1 accumulator
volatile uint32_t pot1ADCcnt = 0;		// Potentiometer 1 counter
volatile uint32_t pot1ADCrdy = 0;		// Potentiometer 1 ready
volatile uint16_t pot2ADCval = 0;		// Potentiometer 2 value
volatile uint32_t pot2ADCacc = 0;		// Potentiometer 2 accumulator
volatile uint32_t pot2ADCcnt = 0;		// Potentiometer 2 counter
volatile uint32_t pot2ADCrdy = 0;		// Potentiometer 2 ready


/* Local function prototypes -------------------------------------------------*/

/* Callback functions --------------------------------------------------------*/

/**
  * @brief  Injected conversion complete callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback (ADC_HandleTypeDef* hadc)
{
	pot1ADCacc += HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_1);
	pot2ADCacc += HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_2);
	pot1ADCcnt++;
	if (pot1ADCcnt >= POT1_AVN)
	  {
		pot1ADCval = pot1ADCacc / POT1_AVN;
		pot1ADCacc = 0;
		pot1ADCcnt = 0;
		pot1ADCrdy++;
	  }
	pot2ADCcnt++;
	if (pot2ADCcnt >= POT2_AVN)
	  {
		pot2ADCval = pot2ADCacc / POT2_AVN;
		pot2ADCacc = 0;
		pot2ADCcnt = 0;
		pot2ADCrdy++;
	  }
}

/* Global functions ----------------------------------------------------------*/

/**
  * @brief  Initializing / starting ADC Channels
  * @param  none
  * @retval none
  */

void InitADCmodule ()
{
   pot1ADCval = 0;
   pot1ADCacc = 0;
   pot1ADCcnt = 0;
   pot1ADCrdy = 0;
   pot2ADCval = 0;
   pot2ADCacc = 0;
   pot2ADCcnt = 0;
   pot2ADCrdy = 0;
   HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
  * @brief  Reading measurement value associated with the ADC Channel.
  *
  *	Checking whether the measurement is ready, and in this case
  *	the measured value is returned. Overrun is also detected.
  *
  * @param  ch - channel index
  * @param  value [out by ref] - measurement value if ready
  * @retval 1 if measurement is ready, 0 not ready, -1 overrun error
  * @note	In the case of overrun: the last value is given and error is cleared
  */

int GetADCChannelValue(uint8_t ch, uint16_t *value)
{
  int status = 0;

	switch (ch)
	  {
		case POT1_CH:
			if (pot1ADCrdy != 0)
			  {
				*value = pot1ADCval;
				status = -1;
				if (pot1ADCrdy == 1) status = 1;
				pot1ADCrdy = 0;
			  }
			break;
		case POT2_CH:
			if (pot2ADCrdy != 0)
			  {
				*value = pot2ADCval;
				status = -1;
				if (pot2ADCrdy == 1) status = 1;
				pot2ADCrdy = 0;
			  }
			break;
		default:;
	  }
	return(status);
}

/************************ (C) COPYRIGHT MTA SZTAKI *****END OF FILE****/
