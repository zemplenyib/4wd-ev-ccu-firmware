/*******************************************************************************
 * File Name:	USART2_itcbuf.c
 * Description:	Driver program for handling USART2 communication using
 *				circular RX and TX buffers with IT
 *******************************************************************************
 *
 * Copyright(c) 2018 MTA SZTAKI
 *
 *******************************************************************************
 */

/*******************************************************************************
 * @file USART2_itcbuf.c
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

/* Constants & Macros --------------------------------------------------------*/

#define	RXBUF_SIZE		128		// Receive Buffer Size
#define	TXBUF_SIZE		2048	// Transmit Buffer Size

/* External variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart3;

/* Private variables ---------------------------------------------------------*/

static uint8_t RxBuf[RXBUF_SIZE];	// Receive Buffer
volatile uint32_t RxIpnt = 0;		// Receive Input Pointer
volatile uint32_t RxOpnt = 0;		// Receive Output Pointer
volatile uint32_t RxCnt = 0;		// Receive Counter
volatile uint32_t RxRdy = 0;		// Receive Ready

static uint8_t TxBuf[TXBUF_SIZE];	// Transmit Buffer
volatile uint32_t TxIpnt = 0;		// Transmit Input Pointer
volatile uint32_t TxOpnt = 0;		// Transmit Output Pointer
volatile uint32_t TxCnt = 0;		// Transmit Counter
volatile uint32_t TxRdy = 0;		// Transmit Ready

/* Local function prototypes -------------------------------------------------*/

/* IT Callback functions -----------------------------------------------------*/

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	  {
		// New data in the RxBuf + RxIpnt position
		RxCnt++;
		if (RxCnt < RXBUF_SIZE)
		  {
			RxIpnt++;
			if (RxIpnt >= RXBUF_SIZE) RxIpnt = 0;
			HAL_UART_Receive_IT(huart,RxBuf + RxIpnt,1);
		  }
		else
		  {
			// Receive process blocked
			COMerrorSignal();
		  }

	  }
}

/**
  * @brief  UART error callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	COMerrorSignal();
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	  {
		// Transmission of 1 character is completed
		TxCnt--;
		TxOpnt++;
		if (TxOpnt >= TXBUF_SIZE) TxOpnt = 0;
		if (TxCnt > 0)
		  {
			// Initiating new transmission: next character in the Buffer
			HAL_UART_Transmit_IT(huart,TxBuf + TxOpnt, 1);
		  }
	  }
}

/* Global functions ----------------------------------------------------------*/

/**
  * @brief  Starting UART Receive Process.
  * @param  None
  * @retval None
  */
void StartUARTCommunication()
{
	RxIpnt = 0;
	RxOpnt = 0;
	RxCnt = 0;
	RxRdy = 0;
	HAL_UART_Receive_IT(&huart3,RxBuf,1);
	TxIpnt = 0;
	TxOpnt = 0;
	TxCnt = 0;
	TxRdy = 0;
}

/**
  * @brief  Testing whether there is any character in the UART Receive Buffer.
  * @param  None
  * @retval RESET - empty, SET - there is at least one character in the buffer
  */
uint32_t TestRxData()
{
	if (RxCnt > 0) return(SET);
	return(RESET);
}

/**
  * @brief  Getting a character from the UART Receive Buffer.
  * @param  None
  * @retval character (null if there is no character in the buffer)
  */
uint8_t GetcRxData()
{
  uint8_t data = 0;
  uint32_t prim;

	if (RxCnt > 0)
	  {
	  	prim = __get_PRIMASK();
	    __disable_irq();
		data = RxBuf[RxOpnt];
		RxOpnt++;
		if (RxOpnt >= RXBUF_SIZE) RxOpnt = 0;
		if (RxCnt >= RXBUF_SIZE)
		  {
			// Restarting receive process
			RxIpnt++;
			if (RxIpnt >= RXBUF_SIZE) RxIpnt = 0;
			HAL_UART_Receive_IT(&huart3,RxBuf + RxIpnt,1);
		  }
		RxCnt--;
	    if (!prim) __enable_irq();
	  }
	return(data);
}

/**
  * @brief  Putting a character to the UART Transmit Buffer.
  * @param  data	the character to be transferred
  * @retval SUCCESS / ERROR
  */
uint32_t PutcTxData(uint8_t data)
{
  uint32_t prim;

	if (TxCnt < TXBUF_SIZE)
	  {
	  	prim = __get_PRIMASK();
	    __disable_irq();
		TxBuf[TxIpnt] = data;
		TxIpnt++;
		if (TxIpnt >= TXBUF_SIZE) TxIpnt = 0;
		TxCnt++;
		if (TxCnt == 1)
		  {
			// Starting Transmit process
			HAL_UART_Transmit_IT(&huart3,TxBuf + TxOpnt,1);
		  }
		if (!prim) __enable_irq();
		return(SUCCESS);
	  }
	else
	  {
		// TX Buffer full: character is discarded
		COMerrorSignal();
		return(ERROR);
	  }
}

/**
  * @brief  Putting a message (NUL-terminated string) to the UART Transmit Buffer.
  * @param  message	the string to be transferred
  * @param  maxsize	maximal size of the message buffer
  * @retval SUCCESS / ERROR
  */
uint32_t PutsTxData(uint8_t *message, int maxsize)
{
	uint32_t index = 0, status = SUCCESS;

	while (index < maxsize && message[index] != NUL_C)
	  {
		status = PutcTxData(message[index]);
		if (status != SUCCESS) break;
		index++;
	  }
	return(status);
}

 /************************ (C) COPYRIGHT MTA SZTAKI *****END OF FILE****/
