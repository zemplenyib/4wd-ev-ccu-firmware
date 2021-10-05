/*******************************************************************************
 * File Name:	USART2_itcbuf.h
 * Description:	Driver program for handling USART2 communication using
 *				circular RX and TX buffers with IT
 *******************************************************************************
 *
 * Copyright(c) 2018 MTA SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file USART2_itcbuf.h
 * @author Alexandros Soumelidis
 * @date 8 Apr 2018
 * @brief Driver program for handling USART2 communication.
 *
 * USART2 RX and TX using circular buffers with IT.
 *                
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"

/* Global function prototypes ------------------------------------------------*/

/**
  * @brief  Starting UART Receive Process.
  * @param  None
  * @retval None
  */
void StartUARTCommunication(void);

/**
  * @brief  Testing whether there is any character in the UART Receive Buffer.
  * @param  None
  * @retval RESET - empty, SET - there is at least one character in the buffer
  */
uint32_t TestRxData(void);

/**
  * @brief  Getting a character from the UART Receive Buffer.
  * @param  None
  * @retval character (null if there is no character in the buffer)
  */
uint8_t GetcRxData(void);

/**
  * @brief  Putting a character to the UART Transmit Buffer.
  * @param  data	the character to be transferred
  * @retval SUCCESS / ERROR
  */
uint32_t PutcTxData(uint8_t data);

/**
  * @brief  Putting a message (NUL-terminated string) to the UART Transmit Buffer.
  * @param  message	the string to be transferred
  * @param  maxsize	maximal size of the message buffer
  * @retval SUCCESS / ERROR
  */
uint32_t PutsTxData(uint8_t *message, int maxsize);

/************************ (C) COPYRIGHT MTA SZTAKI *****END OF FILE************/
