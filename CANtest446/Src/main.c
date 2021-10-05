/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart3_itcbuf.h"
#include "adc1_itinj.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	CTRLLOOP_PER	4				// Control Loop Period (x TIM1)
#define	SWBLOCK_PER		50				// Switch Block Period (x SysTick)
#define	ERRSIGN_PER		100				// Error Signal Duration (x SysTick)
#define	INMSG_SIZE		64				// Input message size

#define	GREEN_LED		1				// 3-color LED identifier
#define	RED_LED			2				// 3-color LED identifier

#define	OPMODE_NUL		0x00			// Operation Mode
#define	OPMODE_S1		0x01			// Operation Mode
#define	OPMODE_S2		0x02			// Operation Mode
#define	OPMODE_S3		0x04			// Operation Mode
#define	OPMODE_REMOTE	0x08			// Operation Mode
#define	OPMODE_BMASK	0x0F			// Operation Mode

#define	CAN_TXID_POT1	0x011			// CAN Arb. ID for Sending POT1
#define	CAN_TXID_POT2	0x012			// CAN Arb. ID for Sending POT2
#define	CAN_TXID_OPMS	0x013			// CAN Arb. ID for Sending Op.Mode

#define	CAN_RXID_REM1	0x621			// CAN Arb. ID for Receiving REM1
#define	CAN_RXID_REM2	0x622			// CAN Arb. ID for Receiving REM2
#define	CAN_RXID_OPMR	0x623			// CAN Arb. ID for Receiving Op.Mode

/* My code from here */
#define LED1 GPIO_PIN_8
#define LED2 GPIO_PIN_6
#define LED3 GPIO_PIN_2
#define LED4 GPIO_PIN_1

#define CAN_MESSAGETYPE_COMMAND 0x00
#define CAN_MESSAGETYPE_CONFIG 0x05
#define CAN_MESSAGETYPE_REFERENCE 0x02

#define CMD_DISCOVER 0x90
#define CMD_DRIVE_STATE 0x50
#define CMD_MODE 0x10
#define CMD_VSRV 0x20
#define CMD_HVDC 0x30

#define VSRV_ON 0x01
#define VSRV_OFF 0x02
#define HVDC_ON 0x01
#define HVDC_OFF 0x02

#define MODE_IDLE 0x11
#define MODE_DRIVE 0x12

#define STATE_STOPPED 0x00
#define STATE_STARTED 0x01

#define CFG_CONTROL_MODE 0x01
#define CONTROL_TORQUE 0x01
#define CONTROL_VELOCITY 0x02

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile uint16_t secCnt = 0;			// Second Counter for SysTick
volatile uint16_t ctrlTick = 0;			// Control Tick Signal
volatile uint16_t ctrlCnt = 0;			// Control Period Counter
volatile uint16_t blinkCnt = 0;			// Blink Period Counter

volatile uint16_t led5signCnt = 0;		// User LED 5 Signal counter
volatile uint16_t led6signCnt = 0;		// User LED 6 Signal counter

volatile uint8_t sw1Block = 0;			// Blocking Period counter for SW1
volatile uint8_t sw2Block = 0;			// Blocking Period counter for SW2
volatile uint8_t sw3Block = 0;			// Blocking Period counter for SW3
volatile uint8_t sw4Block = 0;			// Blocking Period counter for SW4

static uint16_t pot1Val = 0xFFFF;		// Actual value of Potentiometer 1
static uint16_t pot2Val = 0xFFFF;		// Actual value of Potentiometer 2
static uint16_t rem1Val = 0xFFFF;		// Actual Remote value 1
static uint16_t rem2Val = 0xFFFF;		// Actual Remote value 2

static uint8_t inmsgBuf[INMSG_SIZE];	// UART Receive Command Buffer
static uint32_t inmsgPnt = 0;			// UART Receive Command Pointer

TIM_OC_InitTypeDef sConfigOC;			// Config structure for setting TIM3

static uint8_t operMode = OPMODE_NUL;	// Operation Mode
static uint8_t opModeRecv = OPMODE_NUL;	// Operation Mode Received


/* My code from here */
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

/*-----	Memory constants -----------------------------------------------------*/

static const char greetMsg[] = "\r\n\n-- SensAct4 --\n\r";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void Operate3colorLED(int led, uint16_t intens);
void SendOperModeToCAN(void);
void UpdateOperModeLEDs(void);

/* My code from here */
void CAN1_Tx(uint8_t *data, uint8_t DLC, uint16_t ID);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int status;
  uint16_t value;
  char message[16];
  uint8_t rch;
  int ch;
  uint16_t uarg;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

	HAL_TIM_Base_Start_IT(&htim1);
	InitADCmodule();

	StartUARTCommunication();
	PutsTxData((uint8_t*)greetMsg,strlen(greetMsg));

	sprintf(message,"$M%02x\r\n",operMode);
	PutsTxData((uint8_t *)message,strlen(message));
	sprintf(message,"$I\r\n");
	PutsTxData((uint8_t *)message,strlen(message));

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* My code from here */

	uint16_t class;
	uint16_t device;
	uint16_t type;

	uint8_t TxData[8];
	uint16_t id;

    /* Turn on VSRV */
	class = 0x0E;
    device = 0x01;
    type = 0x0;
    id = (class << 7) | (device << 3) | (type); //id = 0x708;

    TxData[0] = CMD_VSRV;
    TxData[1] = VSRV_ON;
	CAN1_Tx(TxData, 2, id);
	HAL_Delay(1500);

    /* Turn on HVDC */
	class = 0x0E;
    device = 0x01;
    type = 0x0;
    id = (class << 7) | (device << 3) | (type); //id = 0x708;

    TxData[0] = CMD_HVDC;
    TxData[1] = HVDC_ON;
	CAN1_Tx(TxData, 2, id);
	HAL_Delay(3000);

	/* CMD discover */
	class = 0x0B;
	device = 0x02; /* FL */
	type = 0x0;
    id = (class << 7) | (device << 3) | (type);

	TxData[0] = CMD_DISCOVER;
	TxData[1] = 0x00;
	CAN1_Tx(TxData,2,id);
	HAL_Delay(100);

	/* CFG_CONTROL_MODE */
	class  = 0x0B;
    device = 0x02;
	type   = CAN_MESSAGETYPE_CONFIG;
	id = (class << 7) | (device << 3) | (type);

	TxData[0] = CFG_CONTROL_MODE;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = CONTROL_VELOCITY;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;
	CAN1_Tx(TxData,8,id);
	HAL_Delay(100);

	/* MODE_DRIVE */
	class  = 0x0B;
	device = 0x02;
	type   = CAN_MESSAGETYPE_COMMAND;
	id = (class << 7) | (device << 3) | (type);

	TxData[0] = CMD_MODE;
	TxData[1] = 0x00;
	TxData[2] = MODE_DRIVE;
	TxData[3] = 0x00;
	CAN1_Tx(TxData,4,id);
	HAL_Delay(100);


	/* STATE_STARTED */
	class  = 0x0B;
	device = 0x02;
	type   = CAN_MESSAGETYPE_COMMAND;
	id = (class << 7) | (device << 3) | (type);

	TxData[0] = 0x50;
	TxData[1] = 0x00;
	TxData[2] = 0x01;
	TxData[3] = 0x00;
	CAN1_Tx(TxData,4,id);
	HAL_Delay(100);

	float iref;
	float vref;

	iref = 2;
	vref = 300;

	TxData[0] = 0x00;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;
/*
	uint8_t data1[sizeof(float)];
	uint8_t data2[sizeof(float)];
	memcpy((void*)data1, (const void*)(&iref), 4);
	memcpy((void*)data2, (const void*)(&vref), 4);

	TxData[0] = data1[0];
	TxData[1] = data1[1];
	TxData[2] = data1[2];
	TxData[3] = data1[3];
	*/
//	memcpy((void*)TxData[0], (const void*)(&data1), 4);
//	memcpy((void*)TxData[4], (const void*)(&data2), 4);


	memcpy((void*)TxData[0], (unsigned char *) (&iref), 4);
	memcpy((void*)TxData[4], (unsigned char *) (&vref), 4);
	//TxData[0] = iref;
	//TxData[4] = vref;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// Waiting for Control Tick signal
	while (ctrlTick == 0) continue;
	ctrlTick = 0;

	/* My code from here */

	/* Velocity Reference */
	class  = 0x0B;
	device = 0x02;
	type   = CAN_MESSAGETYPE_REFERENCE;
	id = (class << 7) | (device << 3) | (type);

	CAN1_Tx(TxData,8,id);
	HAL_GPIO_TogglePin(GPIOC, LED1);
	HAL_Delay(100);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  // Filter Configuration for RX_FIFO0

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
	  /* Filter configuration Error */
	  Error_Handler();
	}

  // Starting CAN

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      /* Start Error */
      Error_Handler();
    }

  // Starting CAN RX process

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
	  /* Notification Error */
	  Error_Handler();
    }

  // Preparing for CAN TX

  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 0;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 250-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4096-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USR_LED2_Pin|USR_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, USR_LED5_Pin|USR_LED3_Pin|USR_LED6_Pin|USR_LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USR_LED2_Pin USR_LED1_Pin */
  GPIO_InitStruct.Pin = USR_LED2_Pin|USR_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USR_SW1_Pin USR_SW4_Pin USR_SW3_Pin USR_SW2_Pin */
  GPIO_InitStruct.Pin = USR_SW1_Pin|USR_SW4_Pin|USR_SW3_Pin|USR_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USR_LED5_Pin USR_LED3_Pin USR_LED6_Pin USR_LED4_Pin */
  GPIO_InitStruct.Pin = USR_LED5_Pin|USR_LED3_Pin|USR_LED6_Pin|USR_LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  char message[8];

	secCnt++;
	if (secCnt == 500)
	  {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);
	  }
	if (secCnt >= 1000)
	  {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
		secCnt = 0;
	  }
	// Handling LED Signals
	if (led5signCnt > 0)
	  {
		if (led5signCnt == 1)
		  {
			HAL_GPIO_WritePin(USR_LED5_GPIO_Port,USR_LED5_Pin,GPIO_PIN_RESET);
		  }
		led5signCnt--;
	  }
	if (led6signCnt > 0)
	  {
		if (led6signCnt == 1)
		  {
			HAL_GPIO_WritePin(USR_LED6_GPIO_Port,USR_LED6_Pin,GPIO_PIN_RESET);
		  }
		led6signCnt--;
	  }
	// Handling Switch (pushbutton) actions
	if (sw1Block > 0)
	  {
		if (sw1Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW1_GPIO_Port,USR_SW1_Pin))
			  {
				// Pushbutton released
			  }
			else
			  {
				// Pushbutton pressed
				if ((operMode  & OPMODE_S1) != 0)
				  {
					operMode &= ~OPMODE_S1;
					HAL_GPIO_WritePin(USR_LED1_GPIO_Port,USR_LED1_Pin,GPIO_PIN_RESET);
				  }
				else
				  {
					operMode |= OPMODE_S1;
					HAL_GPIO_WritePin(USR_LED1_GPIO_Port,USR_LED1_Pin,GPIO_PIN_SET);
				  }
			  }
			sprintf(message,"$M%2X\r\n",operMode);
			PutsTxData((uint8_t *)message,strlen(message));
			SendOperModeToCAN();
		  }
		sw1Block--;
	  }
	if (sw2Block > 0)
	  {
		if (sw2Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW2_GPIO_Port,USR_SW2_Pin))
			  {
				// Pushbutton released
			  }
			else
			  {
				// Pushbutton pressed
				if ((operMode  & OPMODE_S2) != 0)
				  {
					operMode &= ~OPMODE_S2;
					HAL_GPIO_WritePin(USR_LED2_GPIO_Port,USR_LED2_Pin,GPIO_PIN_RESET);
				  }
				else
				  {
					operMode |= OPMODE_S2;
					HAL_GPIO_WritePin(USR_LED2_GPIO_Port,USR_LED2_Pin,GPIO_PIN_SET);
				  }
			  }
			sprintf(message,"$M%2X\r\n",operMode);
			PutsTxData((uint8_t *)message,strlen(message));
			SendOperModeToCAN();
		  }
		sw2Block--;
	  }
	if (sw3Block > 0)
	  {
		if (sw3Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW3_GPIO_Port,USR_SW3_Pin))
			  {
				// Pushbutton released
			  }
			else
			  {
				// Pushbutton pressed
				if ((operMode  & OPMODE_S3) != 0)
				  {
					operMode &= ~OPMODE_S3;
					HAL_GPIO_WritePin(USR_LED3_GPIO_Port,USR_LED3_Pin,GPIO_PIN_RESET);
				  }
				else
				  {
					operMode |= OPMODE_S3;
					HAL_GPIO_WritePin(USR_LED3_GPIO_Port,USR_LED3_Pin,GPIO_PIN_SET);
				  }
			  }
			sprintf(message,"$M%2X\r\n",operMode);
			PutsTxData((uint8_t *)message,strlen(message));
			SendOperModeToCAN();
		  }
		sw3Block--;
	  }
	if (sw4Block > 0)
	  {
		if (sw4Block == 1)
		  {
			if (HAL_GPIO_ReadPin(USR_SW4_GPIO_Port,USR_SW4_Pin))
			  {
				// Pushbutton released
			  }
			else
			  {
				// Pushbutton pressed
				if ((operMode  & OPMODE_REMOTE) != 0)
				  {
					operMode &= ~OPMODE_REMOTE;
					HAL_GPIO_WritePin(USR_LED4_GPIO_Port,USR_LED4_Pin,GPIO_PIN_RESET);
					Operate3colorLED(GREEN_LED, pot1Val);
					Operate3colorLED(RED_LED, pot2Val);
				  }
				else
				  {
					operMode |= OPMODE_REMOTE;
					HAL_GPIO_WritePin(USR_LED4_GPIO_Port,USR_LED4_Pin,GPIO_PIN_SET);
					Operate3colorLED(GREEN_LED, rem1Val);
					Operate3colorLED(RED_LED, rem2Val);
				  }
				sprintf(message,"$M%2X\r\n",operMode);
				PutsTxData((uint8_t *)message,strlen(message));
				SendOperModeToCAN();
			  }
		  }
		sw4Block--;
	  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance == TIM1)
	  {
		ctrlCnt++;
		if (ctrlCnt >= CTRLLOOP_PER)
		 {
			ctrlTick++;
			ctrlCnt = 0;
		 }
	  }
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	  {
		case USR_SW1_Pin:
			sw1Block = SWBLOCK_PER;
			break;
		case USR_SW2_Pin:
			sw2Block = SWBLOCK_PER;
			break;
		case USR_SW3_Pin:
			sw3Block = SWBLOCK_PER;
			break;
		case USR_SW4_Pin:
			sw4Block = SWBLOCK_PER;
			break;
		default:;
	  }
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint16_t uval16;

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
      {
		/* Reception Error */
		Error_Handler();
		return;
      }
	if (RxHeader.IDE == CAN_ID_STD)
      {
		switch (RxHeader.StdId)
	      {
	  		case CAN_RXID_REM1:
	  	  		if (RxHeader.DLC == 2)
	  	  		  {
	  	  			((uint8_t *)&uval16)[0] = RxData[0];
	  	  			((uint8_t *)&uval16)[1] = RxData[1];
	  	  			if (uval16 != rem1Val)
		  	  		  {
						rem1Val = uval16;
						if ((operMode & OPMODE_REMOTE) != 0)
							Operate3colorLED(GREEN_LED, uval16);
		  	  		  }
	  	  		  }
	  	  		break;
	  		case CAN_RXID_REM2:
	  	  		if (RxHeader.DLC == 2)
	  	  		  {
	  	  			((uint8_t *)&uval16)[0] = RxData[0];
	  	  			((uint8_t *)&uval16)[1] = RxData[1];
	  	  			if (uval16 != rem2Val)
		  	  		  {
						rem2Val = uval16;
						if ((operMode & OPMODE_REMOTE) != 0)
							Operate3colorLED(RED_LED, uval16);
		  	  		  }
	  	  		  }
	  	  		break;
	  		case CAN_RXID_OPMR:
	  	  		if (RxHeader.DLC == 1)
	  	  		  {
	  	  			operMode = RxData[0] & OPMODE_BMASK;
	  	  			UpdateOperModeLEDs();
	  	  		  }
	  	  		break;
	  		default:;
		  }
      }
}

/**
  * @brief  Generating USART Communication Error Message.
  *
  * @retval None
  */
void COMerrorSignal()
{
	HAL_GPIO_WritePin(USR_LED5_GPIO_Port,USR_LED5_Pin,GPIO_PIN_SET);
	led5signCnt = ERRSIGN_PER;
}

/**
  * @brief  Generating Command Error Message.
  *
  * @retval None
  */
void CMDerrorSignal()
{
	HAL_GPIO_WritePin(USR_LED6_GPIO_Port,USR_LED6_Pin,GPIO_PIN_SET);
	led6signCnt = ERRSIGN_PER;
}

/**
  * @brief  Updating Operation Mode LEDs.
  * @param  none
  * @retval None
  */
void UpdateOperModeLEDs()
{
	if ((operMode & OPMODE_S1) != 0)
		HAL_GPIO_WritePin(USR_LED1_GPIO_Port,USR_LED1_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(USR_LED1_GPIO_Port,USR_LED1_Pin,GPIO_PIN_RESET);
	if ((operMode & OPMODE_S2) != 0)
		HAL_GPIO_WritePin(USR_LED2_GPIO_Port,USR_LED2_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(USR_LED2_GPIO_Port,USR_LED2_Pin,GPIO_PIN_RESET);
	if ((operMode & OPMODE_S3) != 0)
		HAL_GPIO_WritePin(USR_LED3_GPIO_Port,USR_LED3_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(USR_LED3_GPIO_Port,USR_LED3_Pin,GPIO_PIN_RESET);
	if ((operMode & OPMODE_REMOTE) != 0)
		HAL_GPIO_WritePin(USR_LED4_GPIO_Port,USR_LED4_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(USR_LED4_GPIO_Port,USR_LED4_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  3-color LED operation.
  * @param  led		3-color LED identifier RED or GREEN
  * @param  intens	3-color LED intensity 0 <= intens < 4096
  * @retval None
  */
void Operate3colorLED(int led, uint16_t intens)
{
	sConfigOC.Pulse = intens;
	switch (led)
	  {
		case GREEN_LED:
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			break;
		case RED_LED:
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			break;
		default:;
	  }
}

/**
  * @brief  Sending Operation Mode to CAN
  * @retval None
  */
void SendOperModeToCAN()
{
	TxHeader.StdId = CAN_TXID_OPMS;
	TxHeader.DLC = 1;
	TxData[0] = operMode;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
	  {
		  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		  {
			  /* Transmission request Error */
			  Error_Handler();
		  }
	  }
}

/* My code from here */
void CAN1_Tx(uint8_t *data, uint8_t DLC, uint16_t ID)
{
	TxHeader.DLC = DLC;
	TxHeader.StdId = ID;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0){
		//HAL_GPIO_WritePin(GPIOB, LED3, GPIO_PIN_RESET);
		if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,&TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
		else {
			HAL_GPIO_TogglePin(GPIOB, LED3);//, GPIO_PIN_SET);
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(USR_LED6_GPIO_Port,USR_LED6_Pin,GPIO_PIN_SET);
	led6signCnt = ERRSIGN_PER;

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
