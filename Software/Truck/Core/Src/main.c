/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// NRF commando's
const uint8_t R_REGISTER = 0b00000000;
const uint8_t W_REGISTER = 0b00100000;
const uint8_t R_RX_PAYLOAD = 0b01100001;
const uint8_t W_TXPAYLOAD = 0b10100000;
const uint8_t FLUSH_TX = 0b11100001;
const uint8_t FLUSH_RX = 0b11100010;
const uint8_t REUSE_TX_PL = 0b11100011;
const uint8_t R_RX_PL_WID = 0b01100000;
const uint8_t W_ACK_PAYLOAD = 0b10101000;
const uint8_t W_TX_PAYLOAD_NOACK = 0b10110000;
const uint8_t NOP = 0b11111111;

// NRF registers
const uint8_t CONFIG = 0x00;
const uint8_t EN_AA = 0x01;
const uint8_t EN_RXADDR = 0x02;
const uint8_t SETUP_AW = 0x03;
const uint8_t SETUP_RETR = 0x04;
const uint8_t RF_CH = 0x05;
const uint8_t RF_SETUP = 0x06;
const uint8_t STATUS = 0x07;
const uint8_t OBSERVE_TX = 0x08;
const uint8_t RPD = 0x09;
const uint8_t RX_ADDR_P0 = 0x0A;
const uint8_t RX_ADDR_P1 = 0x0B;
const uint8_t RX_ADDR_P2 = 0x0C;
const uint8_t RX_ADDR_P3 = 0x0D;
const uint8_t RX_ADDR_P4 = 0x0E;
const uint8_t RX_ADDR_P5 = 0x0F;
const uint8_t TX_ADDR = 0x10;
const uint8_t RX_PW_P0 = 0x11;
const uint8_t RX_PW_P1 = 0x12;
const uint8_t RW_PW_P1 = 0x13;
const uint8_t RX_PW_P3 = 0x14;
const uint8_t RX_PW_P4 = 0x15;
const uint8_t RX_PW_P5 = 0x16;
const uint8_t FIFO_STATUS = 0x17;
const uint8_t DYNPD = 0x1C;
const uint8_t FEATURE = 0x1D;

#define servoMinAbs 1565 //minimum waarde servo
#define servoMaxAbs 7826
const uint32_t motorMin = 3130; //minimum waarde motor
const uint32_t motorMax = 6260;
uint32_t servoMin = (servoMinAbs+servoMaxAbs)/2-1000;
uint32_t servoMax = (servoMinAbs+servoMaxAbs)/2+1000;
uint8_t RecData[32] = {0};
uint8_t counter = 0;
uint8_t isDataAvailable (uint8_t);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void Write8_Register_NRF(uint8_t, uint8_t);
uint8_t Read8_Register_NRF(uint8_t);
void CSN_Select(void);
void CSN_Deselect(void);
void Send_Data(uint8_t*);
void NRF_Setup(void);
void WriteX_Register_NRF(uint8_t, uint8_t*, uint8_t);
void Data_Receive(uint8_t*);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_StatusTypeDef xStatus;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
		xStatus = HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    case STDERR_FILENO: /* stderr */
		xStatus = HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  printf("starting up: ");

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //servo
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //motor 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //motor 2

  TIM3->CCR1 = (servoMin+servoMax)/2;
  TIM3->CCR2 = motorMin;
  TIM3->CCR3 = motorMin;

  HAL_Delay(2000);
  NRF_Setup();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //toeter
  TIM4->CCR1 = 50; //50% duty cycle

  uint32_t servoWaarde = 1565; //minimum waarde servo
  uint32_t motorWaarde = 3130; //minimum waarde motor

  printf("OK\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  printf("while loop truck: %02X\r\n", counter++);
	  if (isDataAvailable(0)) {
		  Data_Receive(RecData);
		  printf("Data:%s\r\n", RecData);
		  for (uint8_t i =0; i<32; i++) {
			  printf("%02X ", RecData[i]);
		  } printf("\r\n");
	  } else {
		  printf("Geen data beschikbaar\r\n");
	  }



	  HAL_Delay(100);

/*	  TIM3->CCR2 = motorMin+150;
	  TIM3->CCR3 = motorMin+150;
	  for (uint32_t i = servoMin; i<(servoMax-1500); i++) {
		  //TIM3->CCR1 = i;
		  HAL_Delay(1);
	  }
	  //TIM3->CCR1 = (servoMin+servoMax)/2-1000;
	  TIM3->CCR2 = motorMin;
	  TIM3->CCR3 = motorMin;
	  printf("while loop motor less\r\n");
	  for (uint32_t i = servoMax; i>servoMin; i--) {
	  		  //TIM3->CCR1 = i;
	  		  HAL_Delay(50);
	  	  }*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Prescaler = 22;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62607;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 4696;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, csn_Pin|nrf_ce_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : csn_Pin nrf_ce_Pin */
  GPIO_InitStruct.Pin = csn_Pin|nrf_ce_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Read8_Register_NRF(uint8_t reg) {
	uint8_t data = 0;

	CSN_Select();
	HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi2, &data, 1, 100);
	CSN_Deselect();

	printf("Receive %02X: %02X\r\n", reg, data);
	return 0;
}

void Write8_Register_NRF(uint8_t reg, uint8_t data) {
	uint8_t buf[2];

	buf[0] = reg|W_REGISTER;
	buf[1] = data;

	CSN_Select();
	HAL_SPI_Transmit(&hspi2, buf, 2, 1000);
	CSN_Deselect();
}

void WriteX_Register_NRF(uint8_t reg, uint8_t *data, uint8_t size) {
	CSN_Select();

	uint8_t buf[2];
	buf[0] = reg|1<<5;

	// Pull the CS Pin LOW to select the device
	HAL_SPI_Transmit(&hspi2, buf, 1, 100);
	HAL_SPI_Transmit(&hspi2, data, size, 1000);
	/*
	uint8_t sent[1+size];
	uint8_t cmd = reg|W_REGISTER;
	sent[0] = cmd;

	HAL_StatusTypeDef result;
	result = HAL_SPI_Transmit(&hspi2, sent, size+1, 1000);
	printf("Result: %u\r\n", result);
//	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
//	HAL_SPI_Transmit(&hspi1, data, size, 1000);
*/



	CSN_Deselect();
}

uint8_t* ReadX_Register_NRF(uint8_t reg, uint8_t size) {
	uint8_t data[32] = {0};

	CSN_Select();
	HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
	HAL_SPI_Receive(&hspi2, data, size, 1000);
	CSN_Deselect();

	for (uint8_t i = 0; i<size; i++) {
		printf("%02X ", i);
	}
	printf("\r\n");
	return 0;
}

void Send_Cmd(uint8_t cmd) {
	CSN_Select();
	HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
	CSN_Deselect();
}

void Send_Data(uint8_t *data) {

	uint8_t cmd = W_TXPAYLOAD;
	CSN_Select();
	HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
	HAL_SPI_Transmit(&hspi2, data, 32, 1000);
	CSN_Deselect();
	//Read8_Register_NRF(OBSERVE_TX);

	uint8_t fifostatus = 0; //Read8_Register_NRF(FIFO_STATUS);
	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
		{
			printf("Fifo full\r\n");
			cmd = FLUSH_TX;
			Send_Cmd(cmd);
		} else {
			//printf("empty\r\n");
		}
}


uint8_t isDataAvailable (uint8_t pipenum)
{
	uint8_t status = Read8_Register_NRF(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{

		Write8_Register_NRF(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void NRF_Setup() {
	printf("\r\n\r\nBegin setup NRFFF\r\n");
	CSN_Deselect();
	HAL_GPIO_WritePin(nrf_ce_GPIO_Port, nrf_ce_Pin, GPIO_PIN_RESET);

	Read8_Register_NRF(SETUP_AW);
	Write8_Register_NRF(SETUP_AW, 0x03);
	Read8_Register_NRF(SETUP_AW);

	Write8_Register_NRF(CONFIG, 0);
	Read8_Register_NRF(CONFIG);
	Write8_Register_NRF(CONFIG, 0b10001011); //power up bit
	Read8_Register_NRF(CONFIG);
	HAL_Delay(10);

	Read8_Register_NRF(RF_CH);
	Write8_Register_NRF(RF_CH, 15); //zet channel naar 15
	Read8_Register_NRF(RF_CH);

	Read8_Register_NRF(RF_SETUP);
	Write8_Register_NRF(RF_SETUP, 0x0E);  //set RF_DR_LOW (250kbit) in RF_SETUP 0b00101110
	Read8_Register_NRF(RF_SETUP);

	Write8_Register_NRF(EN_AA, 1);
	Write8_Register_NRF(SETUP_RETR, 0x03);

	ReadX_Register_NRF(TX_ADDR, 5);
	uint8_t addr[5] = {0, 1, 2, 3, 4};
	WriteX_Register_NRF(TX_ADDR, addr, 5);
	ReadX_Register_NRF(TX_ADDR, 5);

	ReadX_Register_NRF(RX_ADDR_P0, 5);
	WriteX_Register_NRF(RX_ADDR_P0, addr,5);
	ReadX_Register_NRF(RX_ADDR_P0, 5);

	Read8_Register_NRF(EN_RXADDR);
	Write8_Register_NRF(EN_RXADDR, 1);
	Read8_Register_NRF(EN_RXADDR);

	Read8_Register_NRF(RX_PW_P0);
	Write8_Register_NRF(RX_PW_P0, 32);
	Read8_Register_NRF(RX_ADDR_P0);



	//set nrf ce voor te enabled te gaan
	HAL_GPIO_WritePin(nrf_ce_GPIO_Port, nrf_ce_Pin, GPIO_PIN_SET);
}

void Data_Receive (uint8_t *data)
{
	CSN_Select();

	uint8_t cmdtosend = R_RX_PAYLOAD;

	HAL_SPI_Transmit(&hspi2, &cmdtosend, 1, 100);
	HAL_SPI_Receive(&hspi2, data, 32, 1000);

	CSN_Deselect();

	HAL_Delay(1);

	//Send_Cmd(FLUSH_RX);
}


void CSN_Select() {
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);
}

void CSN_Deselect() {
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
