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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

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

uint32_t adc_results[4];
uint8_t count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void Write8_Register_NRF(uint8_t, uint8_t);
uint8_t Read8_Register_NRF(uint8_t);
void CSN_Select(void);
void CSN_Deselect(void);
void Send_Data(uint8_t*);
void NRF_Setup(void);
void WriteX_Register_NRF(uint8_t, uint8_t*, uint8_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_StatusTypeDef xStatus;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    case STDERR_FILENO: /* stderr */
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
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

char uart_buf[50];
int uart_buf_len;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //calbrate de ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  NRF_Setup();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(piezo_GPIO_Port, piezo_Pin, GPIO_PIN_SET);

	  if (HAL_GPIO_ReadPin(button_left_GPIO_Port, button_left_Pin) == GPIO_PIN_RESET)
	  	  { printf("Button left\r\n"); }
	  if (HAL_GPIO_ReadPin(button_up_GPIO_Port, button_up_Pin) == GPIO_PIN_RESET)
	  	  { printf("Button up\r\n"); }
	  if (HAL_GPIO_ReadPin(button_down_GPIO_Port, button_down_Pin) == GPIO_PIN_RESET)
	  	  { printf("Button down\r\n"); }
	  if (HAL_GPIO_ReadPin(rotary_switch_GPIO_Port, rotary_switch_Pin) == GPIO_PIN_RESET)
	  	  { printf("Rotary switch\r\n"); }
	  if (HAL_GPIO_ReadPin(joy2_knop_GPIO_Port, joy2_knop_Pin) == GPIO_PIN_RESET)
	  	  { printf("Button joystick 2\r\n"); }

	  HAL_ADC_Start_DMA(&hadc1, adc_results, 4);

	  uint8_t string[32] = "test";
	  printf("Data:%s\r\n", string);
	  	  for (uint8_t i =0; i<32; i++) {
	  		  printf("%02X ", string[i]);
	  	  } printf("\r\n");
	  Send_Data(string);

	  //printf("ADC: %lu R-BO:%lu L-BO:%lu L-RL:%lu\r\n", adc_results[0], adc_results[1], adc_results[2], adc_results[3]);


	  HAL_GPIO_WritePin(piezo_GPIO_Port, piezo_Pin, GPIO_PIN_RESET);
	  printf("While afstandsbediening %02X\r\n", count++);
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(piezo_GPIO_Port, piezo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nrf_CE_Pin|csn_Pin|SPI1_IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : joy2_knop_Pin */
  GPIO_InitStruct.Pin = joy2_knop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(joy2_knop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : piezo_Pin */
  GPIO_InitStruct.Pin = piezo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(piezo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : joy1_knop_Pin */
  GPIO_InitStruct.Pin = joy1_knop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(joy1_knop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nrf_CE_Pin SPI1_IRQ_Pin */
  GPIO_InitStruct.Pin = nrf_CE_Pin|SPI1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : csn_Pin */
  GPIO_InitStruct.Pin = csn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(csn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : rotary1_Pin rotary2_Pin button_pinker_links_Pin button_pinker_rechts_Pin */
  GPIO_InitStruct.Pin = rotary1_Pin|rotary2_Pin|button_pinker_links_Pin|button_pinker_rechts_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : rotary_switch_Pin button_left_Pin button_up_Pin button_down_Pin */
  GPIO_InitStruct.Pin = rotary_switch_Pin|button_left_Pin|button_up_Pin|button_down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Read8_Register_NRF(uint8_t reg) {
	uint8_t data = 0;

	CSN_Select();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi1, &data, 1, 100);
	CSN_Deselect();

	printf("Receive %02X: %02X\r\n", reg, data);
	return data;
}

void Write8_Register_NRF(uint8_t reg, uint8_t data) {
	uint8_t buf[2];

	buf[0] = reg|W_REGISTER;
	buf[1] = data;

	CSN_Select();
	HAL_SPI_Transmit(&hspi1, buf, 2, 1000);
	CSN_Deselect();
}

void WriteX_Register_NRF(uint8_t reg, uint8_t *data, uint8_t size) {
	CSN_Select();

	uint8_t cmd = reg|W_REGISTER;
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, size, 1000);
	CSN_Deselect();
}

uint8_t* ReadX_Register_NRF(uint8_t reg, uint8_t size) {
	uint8_t data[32] = {0};

	CSN_Select();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Receive(&hspi1, data, size, 1000);
	CSN_Deselect();

	for (uint8_t i = 0; i<size; i++) {
		printf("%02X ", i);
	}
	printf("\r\n");
	return data;
}

void Send_Cmd(uint8_t cmd) {
	CSN_Select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	CSN_Deselect();
}

void Send_Data(uint8_t *data) {

	uint8_t cmd = W_TXPAYLOAD;
	CSN_Select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, 32, 10000);
	CSN_Deselect();

	printf("observe tx ");
	Read8_Register_NRF(OBSERVE_TX);

	uint8_t fifostatus = Read8_Register_NRF(FIFO_STATUS);
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

void NRF_Setup() {
	printf("\r\nBegin setup NRF\r\n");
	CSN_Deselect();
	HAL_GPIO_WritePin(nrf_CE_GPIO_Port, nrf_CE_Pin, GPIO_PIN_RESET);

	printf("SETUP_AW: ");
	Write8_Register_NRF(SETUP_AW, 0x03);
	Read8_Register_NRF(SETUP_AW);

	printf("CONFIG: ");
	Write8_Register_NRF(CONFIG, 0b00001010); //power up bit
	Read8_Register_NRF(CONFIG);
	HAL_Delay(10);

	printf("RF_CH: ");
	Write8_Register_NRF(RF_CH, 15); //zet channel naar 15
	Read8_Register_NRF(RF_CH);

	printf("RF_SETUP: ");
	Write8_Register_NRF(RF_SETUP, 0x0E);  //set RF_DR_LOW (250kbit) in RF_SETUP 0b00101110
	Read8_Register_NRF(RF_SETUP);

	Write8_Register_NRF(EN_AA, 1);
	Write8_Register_NRF(SETUP_RETR, 0x03);

	uint8_t addr[5] = {0, 1, 2, 3, 4};
	printf("TX_ADDR: ");
	WriteX_Register_NRF(TX_ADDR, addr, 5);
	ReadX_Register_NRF(TX_ADDR, 5);

	printf("RX_ADDR: ");
	WriteX_Register_NRF(RX_ADDR_P0, addr, 5);
	ReadX_Register_NRF(RX_ADDR_P0, 5);


	//set nrf ce voor in TX modus te gaan
	HAL_GPIO_WritePin(nrf_CE_GPIO_Port, nrf_CE_Pin, GPIO_PIN_SET);
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
