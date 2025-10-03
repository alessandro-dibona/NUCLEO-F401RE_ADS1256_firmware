/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for Command_Handler */
osThreadId_t Command_HandlerHandle;
const osThreadAttr_t Command_Handler_attributes = {
  .name = "Command_Handler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UART_TX_Task */
osThreadId_t UART_TX_TaskHandle;
const osThreadAttr_t UART_TX_Task_attributes = {
  .name = "UART_TX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_RX_Task */
osThreadId_t UART_RX_TaskHandle;
const osThreadAttr_t UART_RX_Task_attributes = {
  .name = "UART_RX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPI_RX_Task */
osThreadId_t SPI_RX_TaskHandle;
const osThreadAttr_t SPI_RX_Task_attributes = {
  .name = "SPI_RX_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for txDataQueue */
osMessageQueueId_t txDataQueueHandle;
const osMessageQueueAttr_t txDataQueue_attributes = {
  .name = "txDataQueue"
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for UART_RX_Done_Sem */
osSemaphoreId_t UART_RX_Done_SemHandle;
const osSemaphoreAttr_t UART_RX_Done_Sem_attributes = {
  .name = "UART_RX_Done_Sem"
};
/* Definitions for DRDY_Signal_Sem */
osSemaphoreId_t DRDY_Signal_SemHandle;
const osSemaphoreAttr_t DRDY_Signal_Sem_attributes = {
  .name = "DRDY_Signal_Sem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
void StartCommand_Handler(void *argument);
void StartUART_TX_Task(void *argument);
void StartUART_RX_Task(void *argument);
void StartSPI_RX_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_PARAM_BYTES 8
#define UART_RX_BUFFER_SIZE 32

#define STARTBYTE 0xAA
#define ACK       0x06
#define NAK       0x15

#define WAKEUP    0xE0
#define SELFCAL   0xE1
#define SELFOCAL  0xE2
#define SELFGCAL  0xE3
#define SYSOCAL   0xE4
#define SYSGCAL   0xE5
#define TRIGGER   0xE6
#define STANDBY   0xE7
#define RESET     0xE8
#define ACALON    0xE9
#define ACALOFF   0xEA
#define BUFEN     0xEB
#define BUFDIS    0xEC
#define MUX       0xED
#define PGA       0xEE
#define DRATE     0xEF
#define OFCW      0xF0
#define OFCR      0xF1
#define FSCW      0xF2
#define FSCR      0xF3
#define AVERAGE   0xF4
#define READREGS  0xF5
#define READCAL   0xF6
#define DUMMY     0xF7

#define DRATE_30000SPS       0b11110000
#define DRATE_15000SPS       0b11100000
#define DRATE_7500SPS        0b11010000
#define DRATE_3750SPS        0b11000000
#define DRATE_2000SPS        0b10110000
#define DRATE_1000SPS        0b10100001
#define DRATE_500SPS         0b10010010
#define DRATE_100SPS         0b10000010
#define DRATE_60SPS          0b01110010
#define DRATE_50SPS          0b01100011
#define DRATE_30SPS          0b01010011
#define DRATE_25SPS          0b01000011
#define DRATE_15SPS          0b00110011
#define DRATE_10SPS          0b00100011
#define DRATE_5SPS           0b00010011
#define DRATE_2_5SPS         0b00000011

#define SUCCESS              0x00
#define INVALID_COMMAND      0x01
#define INVALID_PARAM_VALUE  0x02
#define INVALID_PARAM_COUNT  0x03
#define PARAM_NOT_SUPPORTED  0x04

#define ADS1256_REG_STATUS   0x00
#define ADS1256_REG_MUX      0x01
#define ADS1256_REG_ADCON    0x02
#define ADS1256_REG_DRATE    0x03
#define ADS1256_REG_IO       0x04
#define ADS1256_REG_OFC0     0x05
#define ADS1256_REG_OFC1     0x06
#define ADS1256_REG_OFC2     0x07
#define ADS1256_REG_FSC0     0x08
#define ADS1256_REG_FSC1     0x09
#define ADS1256_REG_FSC2     0x0A

#define ADS1256_COM_WAKEUP   0x00
#define ADS1256_COM_RDATA    0x01
#define ADS1256_COM_RDATAC   0x03
#define ADS1256_COM_SDATAC   0x0F
#define ADS1256_COM_RREG     0x10
#define ADS1256_COM_WREG     0x50
#define ADS1256_COM_SELFCAL  0xF0
#define ADS1256_COM_SELFOCAL 0xF1
#define ADS1256_COM_SELFGCAL 0xF2
#define ADS1256_COM_SYSOCAL  0xF3
#define ADS1256_COM_SYSGCAL  0xF4
#define ADS1256_COM_SYNC     0xFC
#define ADS1256_COM_STANDBY  0xFD
#define ADS1256_COM_RESET    0xFE
#define ADS1256_COM_SNCWKP   0xFF

uint8_t UART_RX_buf[UART_RX_BUFFER_SIZE];
uint8_t UART_RX_bytes_received = 0;

volatile uint8_t read_data_on_drdy = false;			// enable (true) or disable (false) the call of StartSPI_RX_Task on DRDY; checked in HAL_GPIO_EXTI_Callback
volatile int32_t adccode;							// ultima conversione ADC, aggiornato nella nella ADS1256ReadData. Il dato intero va scalato per avere il valore in volt.
volatile int32_t adccodesum = 0;					// accumulatore delle conversioni ADC, usata per calcolare il valor medio
volatile uint8_t nmeas = 0;							// numero di conversioni accumulate in adccodesum
volatile uint8_t naverage = 5;                      // numero di conversioni per il calcolo della media
volatile double dvoltage = 0.;						// tensione media misurata, tiene conto di pga
volatile uint8_t pga = 0;

const double vres_lut[] = {5.960465188081883e-7, 2.980232594040941e-7, 1.490116297020471e-7, 7.450581485102354e-8, 3.725290742551177e-8, 1.862645371275588e-8, 9.313226856377942e-9};


typedef struct {
    uint8_t command_id;               // 1 byte: Il comando
    uint8_t statuscode;				  // 1 byte: 0x00 se SUCCESS, altrimenti un codice di errore
    uint8_t nbytes;					  // 1 byte: Numero di bytes del payload
    uint8_t payload[MAX_PARAM_BYTES]; // Buffer per MAX_PARAM_BYTES parametri
} CommsFrame_t;

int _write(int file, char *ptr, int len)
{
	// implementazione _write per serial wire debug usata da printf e puts
	// usa la periferica ITM progettata per serial wire debugging che utilizza il pin SWO
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case BTN_BLU_Pin:

		printf("BTN_BLU_Pin IRQ\n");
		htim2.Instance->CR1 |= TIM_CR1_CEN;				// triggering di one-shot timer per blink del LED onboard
		break;

	case DRDY_Pin:										// con questo IRQ l'ADS1256 ci informa la conversione è completa

		htim2.Instance->CR1 |= TIM_CR1_CEN;				// triggering di one-shot timer per blink del LED onboard

		if (read_data_on_drdy)
			osSemaphoreRelease(DRDY_Signal_SemHandle);	// se read_data_on_drdy = true allora sblocco il task per la lettura del dato sulla SPI

		break;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)	// chiamata quando la UART va in idle, quando il buffer è pieno o quando il buffer è mezzo pieno
{
	if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_HT)			// filtro IRQ: ignoro le chiamate per buffer mezzo pieno.
	{
		UART_RX_bytes_received = Size;										// terminata la ricezione aggiorno la variabile globale con il n. di caratteri ricevuti;
		osSemaphoreRelease(UART_RX_Done_SemHandle);							// comunico al task di ricezione che è stato ricevuto un nuovo buffer
	}
}

void ShowFrame(CommsFrame_t* pframe)
{
	printf("command_id: 0x%X\n", pframe->command_id);

	for (uint8_t i=0; i<MAX_PARAM_BYTES; i++)
	{
		printf("B[%d]:      0x%X\n", i, pframe->payload[i]);
	}
}

void SendResponse(uint8_t command_id, uint8_t statuscode, uint8_t nbytes, uint8_t* payload)
{
	CommsFrame_t frame_out;

	frame_out.command_id = command_id;
	frame_out.statuscode = statuscode;
	frame_out.nbytes = nbytes;

	if (nbytes > 0)
	{
		memcpy(&frame_out.payload, payload, nbytes);
	}

	osMessageQueuePut(txDataQueueHandle, &frame_out, 0, 0);
}


void delay_busywait(uint32_t count) {								// impostare TIM9 con un prescaler di 84-1 per avere un clock da 1 us/count

    __HAL_TIM_SET_COUNTER(&htim9, 0);								// Reimposta il contatore del timer
    __HAL_TIM_ENABLE(&htim9);										// Avvia il timer

    while (__HAL_TIM_GET_COUNTER(&htim9) < count);					// Attendi in un ciclo finché il contatore non raggiunge il valore desiderato

    __HAL_TIM_DISABLE(&htim9);										// Ferma il timer
}


uint8_t ADS1256_ReadRegister(uint8_t registerID)
{
	  uint8_t tx_data[2];
	  uint8_t rx_data[1];

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);						// abbasso CS

	  tx_data[0] = ADS1256_COM_RREG | registerID;									// comando RREG e offset registro
	  tx_data[1] = 0x00;															// numero di bytes da leggere - 1 (impostare a zero per leggere 1 byte)

	  HAL_SPI_Transmit(&hspi3, tx_data, 2, HAL_MAX_DELAY);							// invio due bytes

	  delay_busywait(10ul);															// attendo 10 us (vedi configurazione di TIM9

	  HAL_SPI_Receive(&hspi3, rx_data, 1, HAL_MAX_DELAY);							// ricevo un byte via SPI

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);						// alzo CS

	  delay_busywait(5ul);

	  return rx_data[0];

}

void ADS1256_WriteRegister(uint8_t registerID, uint8_t value)
{
	  uint8_t tx_data[3];

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);						// abbasso CS

	  tx_data[0] = ADS1256_COM_WREG | registerID;									// comando WREG e offset registro
	  tx_data[1] = 0x00;															// numero di bytes da scrivere - 1 (impostare a zero per scrivere 1 byte)
	  tx_data[2] = value;

	  HAL_SPI_Transmit(&hspi3, tx_data, 3, HAL_MAX_DELAY);							// invio tre bytes

	  delay_busywait(10ul);															// attendo 10 us (vedi configurazione di TIM9)

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);						// alzo CS

	  delay_busywait(10ul);															// attendo 10 us (vedi configurazione di TIM9)
}

void ADS1256_SendCommand(uint8_t commandID)
{
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);						// abbasso CS

	  HAL_SPI_Transmit(&hspi3, &commandID, 1, HAL_MAX_DELAY);						// invio un byte

	  delay_busywait(5ul);															// attendo 5 us (vedi configurazione di TIM9)

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);						// alzo CS

	  delay_busywait(5ul);															// attendo 5 us (vedi configurazione di TIM9)
}

void ADS1256_ReadData()
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);						// abbasso CS

	uint8_t rdatacommand = ADS1256_COM_RDATA;
	HAL_SPI_Transmit(&hspi3, &rdatacommand, 1, HAL_MAX_DELAY);						// trasmetto il comando RDATA

	delay_busywait(10);																// attendo 10 us (vedi configurazione di TIM9)

	uint8_t adcbyte[3];
	HAL_SPI_Receive(&hspi3, adcbyte, 3, HAL_MAX_DELAY);								// ricevo tre bytes via SPI

	adccode = (int32_t)((uint32_t)adcbyte[0] << 24) | ((uint32_t)adcbyte[1] << 16) | ((uint32_t)adcbyte[2] << 8);		// riordino e allineo a sinistra i bytes
	adccode = adccode >> 8;																								// sposto 8 bit a destra con estensione del segno

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);							// alzo CS

	delay_busywait(5);
}

void ADS1256_Trigger()
{
	TIM3->CR1 |= TIM_CR1_CEN;
	read_data_on_drdy = true;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  printf("\n\n\n\nRunning...\n");

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART_RX_buf, UART_RX_BUFFER_SIZE);		// La prima volta viene invocata qui e mette l'UART in attesa di un messaggio.
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	// Quando arriva il messaggio viene chiamata la callback HAL_UARTEx_RxEventCallback
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	// e la ricezione via UART non riprende finchè la funzione
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	// HAL_UARTEx_ReceiveToIdle_DMA non viene nuovamente invocata.
  printf("UART receive started.\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UART_RX_Done_Sem */
  UART_RX_Done_SemHandle = osSemaphoreNew(1, 1, &UART_RX_Done_Sem_attributes);

  /* creation of DRDY_Signal_Sem */
  DRDY_Signal_SemHandle = osSemaphoreNew(1, 0, &DRDY_Signal_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of txDataQueue */
  txDataQueueHandle = osMessageQueueNew (32, sizeof(CommsFrame_t), &txDataQueue_attributes);

  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (16, sizeof(CommsFrame_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Command_Handler */
  Command_HandlerHandle = osThreadNew(StartCommand_Handler, NULL, &Command_Handler_attributes);

  /* creation of UART_TX_Task */
  UART_TX_TaskHandle = osThreadNew(StartUART_TX_Task, NULL, &UART_TX_Task_attributes);

  /* creation of UART_RX_Task */
  UART_RX_TaskHandle = osThreadNew(StartUART_RX_Task, NULL, &UART_RX_Task_attributes);

  /* creation of SPI_RX_Task */
  SPI_RX_TaskHandle = osThreadNew(StartSPI_RX_Task, NULL, &SPI_RX_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);		// IMPORTANTE: il canale di out va inserito a mano!!!!

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5;
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
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);		// IMPORTANTE: il canale di out va inserito a mano!!!!

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 83;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN_BLU_Pin */
  GPIO_InitStruct.Pin = BTN_BLU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_BLU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCommand_Handler */
/**
  * @brief  Function implementing the Command_Handler thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommand_Handler */
void StartCommand_Handler(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	CommsFrame_t frame_in;
	uint8_t reg;
	uint8_t payload[MAX_PARAM_BYTES];

	for(;;)
	{
		osStatus_t status = osMessageQueueGet(CommandQueueHandle, &frame_in, NULL, osWaitForever);

		if (status == osOK)
		{
			switch (frame_in.command_id)
			{
			case WAKEUP:
				printf("Comando WAKEUP\n");

				ADS1256_SendCommand(ADS1256_COM_WAKEUP);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case SELFCAL:
				printf("Comando SELFCAL\n");

				ADS1256_SendCommand(ADS1256_COM_SELFCAL);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case SELFOCAL:
				printf("Comando SELFOCAL\n");

				ADS1256_SendCommand(ADS1256_COM_SELFOCAL);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case SELFGCAL:
				printf("Comando SELFGCAL\n");

				ADS1256_SendCommand(ADS1256_COM_SELFGCAL);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case SYSOCAL:
				printf("Comando SYSOCAL\n");

				ADS1256_SendCommand(ADS1256_COM_SYSOCAL);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case SYSGCAL:
				printf("Comando SYSGCAL\n");

				ADS1256_SendCommand(ADS1256_COM_SYSGCAL);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case TRIGGER:
				printf("Comando TRIGGER\n");

				ADS1256_Trigger();

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case STANDBY:
				printf("Comando STANDBY\n");

				ADS1256_SendCommand(ADS1256_COM_STANDBY);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case RESET:
				printf("Comando RESET\n");

				ADS1256_SendCommand(ADS1256_COM_RESET);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case ACALON:
				printf("Comando ACALON\n");

				reg = ADS1256_ReadRegister(ADS1256_REG_STATUS) | 0b00000100;
				ADS1256_WriteRegister(ADS1256_REG_STATUS, reg);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case ACALOFF:
				printf("Comando ACALOFF\n");

				reg = ADS1256_ReadRegister(ADS1256_REG_STATUS) & 0b11111011;
				ADS1256_WriteRegister(ADS1256_REG_STATUS, reg);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case BUFEN:
				printf("Comando BUFEN\n");

				reg = ADS1256_ReadRegister(ADS1256_REG_STATUS) | 0b00000010;
				ADS1256_WriteRegister(ADS1256_REG_STATUS, reg);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case BUFDIS:
				printf("Comando BUFDIS\n");

				reg = ADS1256_ReadRegister(ADS1256_REG_STATUS) & 0b11111101;
				ADS1256_WriteRegister(ADS1256_REG_STATUS, reg);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case MUX:
				printf("Comando MUX\n");

				reg = frame_in.payload[0];
				ADS1256_WriteRegister(ADS1256_REG_MUX, reg);

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case PGA:
				printf("Comando PGA\n");

				uint8_t pga_in = frame_in.payload[0];

				// Validate parameter
				// pga value   gain
				//     0         1
				//     1         2
				//     2         4
				//     3         8
				//     4        16
				//     5        32
				//     6        64
				//     7        64

				if (pga_in > 7)
				{
					SendResponse(frame_in.command_id, INVALID_PARAM_VALUE, 0, NULL);
				}
				else
				{
					pga = pga_in;
					reg = (ADS1256_ReadRegister(ADS1256_REG_ADCON) & 0b11111000) | pga;
					ADS1256_WriteRegister(ADS1256_REG_ADCON, reg);

					SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				}

				break;
			case DRATE:
				printf("Comando DRATE\n");

				uint8_t drate = frame_in.payload[0];

				// Validate parameter
				switch(drate)
				{
				case DRATE_30000SPS:
				case DRATE_15000SPS:
				case DRATE_7500SPS:
				case DRATE_3750SPS:
				case DRATE_2000SPS:
				case DRATE_1000SPS:
				case DRATE_500SPS:
				case DRATE_100SPS:
				case DRATE_60SPS:
				case DRATE_50SPS:
				case DRATE_30SPS:
				case DRATE_25SPS:
				case DRATE_15SPS:
				case DRATE_10SPS:
				case DRATE_5SPS:
				case DRATE_2_5SPS:

					ADS1256_WriteRegister(ADS1256_REG_DRATE, drate);

					SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
					break;
				default:
					SendResponse(frame_in.command_id, INVALID_PARAM_VALUE, 0, NULL);
				}

				break;
			case OFCW:
				printf("Comando OFCW\n");

				uint8_t ofc1 = frame_in.payload[0];
				uint8_t ofc2 = frame_in.payload[1];
				uint8_t ofc3 = frame_in.payload[2];

				printf("Parametri: 0x%X 0x%X 0x%X\n", ofc1, ofc2, ofc3);

				// TODO: Process command
				// ...

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case OFCR:
				printf("Comando OFCR\n");

				// OFC0, OFC1, OFC2
				payload[0] = ADS1256_ReadRegister(ADS1256_REG_OFC0);
				payload[1] = ADS1256_ReadRegister(ADS1256_REG_OFC1);
				payload[2] = ADS1256_ReadRegister(ADS1256_REG_OFC2);

				SendResponse(frame_in.command_id, SUCCESS, 3, payload);

				break;
			case FSCW:
				printf("Comando FSCW\n");

				uint8_t fsc1 = frame_in.payload[0];
				uint8_t fsc2 = frame_in.payload[1];
				uint8_t fsc3 = frame_in.payload[2];

				printf("Parametri: 0x%X 0x%X 0x%X\n", fsc1, fsc2, fsc3);

				// TODO: Process command
				// ...

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case FSCR:
				printf("Comando FSCR\n");

				// FSC0, FSC1, FSC2
				payload[0] = ADS1256_ReadRegister(ADS1256_REG_FSC0);
				payload[1] = ADS1256_ReadRegister(ADS1256_REG_FSC1);
				payload[2] = ADS1256_ReadRegister(ADS1256_REG_FSC2);

				SendResponse(frame_in.command_id, SUCCESS, 3, payload);

				break;
			case AVERAGE:
				printf("Comando AVERAGE\n");

				uint8_t navg = frame_in.payload[0];
				printf("Parametro: 0x%X\n", navg);

				naverage = frame_in.payload[0];

				SendResponse(frame_in.command_id, SUCCESS, 0, NULL);
				break;
			case READREGS:
				printf("Comando READREGS\n");

				// STATUS, MUX, ADCON, DRATE, IO, AVERAGE
				payload[0] = ADS1256_ReadRegister(ADS1256_REG_STATUS);
				payload[1] = ADS1256_ReadRegister(ADS1256_REG_MUX);
				payload[2] = ADS1256_ReadRegister(ADS1256_REG_ADCON);
				payload[3] = ADS1256_ReadRegister(ADS1256_REG_DRATE);
				payload[4] = ADS1256_ReadRegister(ADS1256_REG_IO);
				payload[5] = naverage;

				SendResponse(frame_in.command_id, SUCCESS, 6, payload);

				break;
			case READCAL:
				printf("Comando READCAL\n");

				// OFC0, OFC1, OFC2, FSC0, FSC1, FSC2
				payload[0] = ADS1256_ReadRegister(ADS1256_REG_OFC0);
				payload[1] = ADS1256_ReadRegister(ADS1256_REG_OFC1);
				payload[2] = ADS1256_ReadRegister(ADS1256_REG_OFC2);
				payload[3] = ADS1256_ReadRegister(ADS1256_REG_FSC0);
				payload[4] = ADS1256_ReadRegister(ADS1256_REG_FSC1);
				payload[5] = ADS1256_ReadRegister(ADS1256_REG_FSC2);

				SendResponse(frame_in.command_id, SUCCESS, 6, payload);

				break;
			case DUMMY:
				printf("Comando DUMMY\n");

				read_data_on_drdy = !read_data_on_drdy;

				break;

			default:
				SendResponse(frame_in.command_id, INVALID_COMMAND, 0, NULL);
				printf("StartCommand_Handler ERROR: messaggio sconosciuto.\n");
			}
		}
		else
		{
			printf("StartCommand_Handler ERROR: osMessageQueueGet(CommandQueue) fail.\n");
		}


	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUART_TX_Task */
/**
* @brief Function implementing the UART_TX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_TX_Task */
void StartUART_TX_Task(void *argument)
{
  /* USER CODE BEGIN StartUART_TX_Task */

	CommsFrame_t frame;							// command_id, statuscode, nbytes, [payload]
	uint8_t buf[sizeof(CommsFrame_t) + 4];		// STARTBYTE STATUSCODE COMMAND [B1, B2, ...] CHECKSUM

  /* Infinite loop */
	for(;;)
	{
		osStatus_t status = osMessageQueueGet(txDataQueueHandle, &frame, NULL, osWaitForever);

		if (status == osOK)
		{
			uint8_t bytes_to_transmit;

			if ((frame.command_id == ACK) || (frame.command_id == NAK))			// ACK o NAK: il buffer è formato da un solo byte
			{
				buf[0] = frame.command_id;
				bytes_to_transmit = 1;
			}
			else																// messaggio completo: preparo il buffer
			{
				buf[0] = STARTBYTE;
				buf[1] = frame.statuscode;
				buf[2] = frame.command_id;
				buf[3] = frame.nbytes;

				memcpy(&buf[4], &frame.payload,frame.nbytes);

				uint8_t checksum = 0;

				for(uint8_t i = 0; i < (4+frame.nbytes); i++)
				{
					checksum += buf[i];
				}

				buf[4+frame.nbytes] = checksum;
																				// |--------------- 4 bytes ---------------|  |---- nbytes -----|  |1 byte|
				bytes_to_transmit = 4 + frame.nbytes + 1;						// startbyte, statuscode, command_id, nbytes, [byte1, byte2, ...], checksum
																				// [0]        [1]         [2]         [3]     [4]                  [4+nbytes]
			}

			HAL_UART_Transmit(&huart2, buf, bytes_to_transmit, 100);			// invio il buffer all'UART
		}
		else
		{
			printf("StartUART_TX_Task ERROR: osMessageQueueGet(txDataQueue) fail.\n");
		}

	}
  /* USER CODE END StartUART_TX_Task */
}

/* USER CODE BEGIN Header_StartUART_RX_Task */
/**
* @brief Function implementing the UART_RX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_RX_Task */
void StartUART_RX_Task(void *argument)
{
  /* USER CODE BEGIN StartUART_RX_Task */
  /* Infinite loop */

  uint8_t checksum;
  CommsFrame_t frame;

  for(;;)
  {
	  osSemaphoreAcquire(UART_RX_Done_SemHandle, osWaitForever);					// il task si blocca in attesa di un messaggio nella coda dei comandi

	  // struttura di UART_RX_buf: STARTBYTE COMMAND_ID B1    B2   ...   CHECKSUM
	  //                           [0]       [1]        [2]   [3]  ...   [UART_RX_bytes_received-1]

	  if (UART_RX_bytes_received > 0)
	  {
		  if (UART_RX_buf[0] != STARTBYTE)
		  {
			  // metto NAK in coda txDataQueue
			  frame.command_id = NAK;
			  osMessageQueuePut(txDataQueueHandle, &frame, 0, 0);
		  }
		  else
		  {
			  // calcolo la checksum
			  checksum = 0;

			  for (int i=0; i<(UART_RX_bytes_received-1); i++)
				  checksum += UART_RX_buf[i];

			  if (checksum != UART_RX_buf[UART_RX_bytes_received-1])
			  {
				  // metto NAK in coda txDataQueue
				  frame.command_id = NAK;
				  osMessageQueuePut(txDataQueueHandle, &frame, 0, 0);
			  }
			  else
			  {
				  // il comando è OK
				  // metto ACK in coda txDataQueue
				  frame.command_id = ACK;
				  osMessageQueuePut(txDataQueueHandle, &frame, 0, 0);

				  // metto il comando in coda CommandQueue
				  frame.command_id = UART_RX_buf[1];										// copio il command_id
				  frame.nbytes = UART_RX_buf[2];											// copio il numbytes
				  memcpy(&frame.payload, &UART_RX_buf[2], UART_RX_bytes_received-3);		// copio solo il payload

				  osMessageQueuePut(CommandQueueHandle, &frame, 0, 0);
			  }
		  }
	  }

	  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART_RX_buf, UART_RX_BUFFER_SIZE);		// invoco nuovamente HAL_UARTEx_ReceiveToIdle_DMA per ricevere il prossimo messaggio
  }
  /* USER CODE END StartUART_RX_Task */
}

/* USER CODE BEGIN Header_StartSPI_RX_Task */
/**
* @brief Function implementing the SPI_RX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPI_RX_Task */
void StartSPI_RX_Task(void *argument)
{
  /* USER CODE BEGIN StartSPI_RX_Task */
  /* Infinite loop */

	CommsFrame_t frame;

	for(;;)
	{
		osSemaphoreAcquire(DRDY_Signal_SemHandle, osWaitForever);		// Attendi indefinitamente che il semaforo venga rilasciato (cioè quando DRDY transisce low)

		ADS1256_ReadData();

		if (nmeas < naverage)
		{
			nmeas++;
			adccodesum += adccode;
		}
		else
		{
			read_data_on_drdy = false;

			dvoltage = (double)adccodesum/(double)nmeas * vres_lut[pga];

			frame.command_id = TRIGGER;
			frame.statuscode = SUCCESS;
			frame.nbytes = sizeof(double);

			memcpy(frame.payload, (const void *)&dvoltage, sizeof(double));

			osMessageQueuePut(txDataQueueHandle, &frame, 0, 0);

			nmeas = 0;
			adccodesum = 0;
		}
	}
  /* USER CODE END StartSPI_RX_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
