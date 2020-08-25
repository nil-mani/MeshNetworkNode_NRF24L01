/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include "MY_NRF24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REQ_DIS 10

//msg ID's for msg struct
#define SETUP_MSG 0
#define NEIGHBOR_MSG 1
#define ROUTE_MSG 2
#define PROX_MSG 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void generateMsg(uint8_t msgID, uint32_t msgData);
void TimerDelay(uint32_t Sec);
void startSensor(void);
uint32_t calculateDistance(void);
void strtForward(void);
void reverse(void);
void turnLeft(void);
void maneuverObstacle();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//////////////////////////
uint8_t nodeID = 1;  ///// THIS VALUE MUST BE CHANGED WHENEVER UPLOADING TO A NEW BOARD
//uint8_t nodeID = 2;
//uint8_t nodeID = 3;
//uint8_t nodeID = 4;
//////////////////////////
uint64_t pipeAddrs = 0x11223344AA;
uint32_t myTxMessage[32];
uint32_t myRxMessage[32];

const float speedOfSound =0.0343/2;
uint32_t distance;
char usartRead[100];



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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  NRF24_begin(GPIOB,CSNpin_Pin, CEpin_Pin, hspi1);
  nrf24_DebugUART_Init(huart2);

  printRadioSettings();


  NRF24_stopListening();

  NRF24_openReadingPipe(1, pipeAddrs);
  NRF24_setAutoAck(false);
  NRF24_setChannel(52);
  NRF24_setPayloadSize(32);
  NRF24_startListening();

  //initialize Proximity sensor

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 	  //Start of Node code
	         //puts the NRF in listening mode

	  	    if(NRF24_available()){
	  	  		  NRF24_read(myRxMessage,32);

	  	  		  myRxMessage[32] = '\r'; myRxMessage[32+1] = '\n';
	  	  		  sprintf(usartRead,"Recieved Message! MSGID:%ld, Distance =%lu, Source Node: %ld, this NodeID = %d\n\r",myRxMessage[0], myRxMessage[3], myRxMessage[1], nodeID);
	  	  		  HAL_UART_Transmit(&huart2, (uint8_t*)usartRead, strlen(usartRead), 100);
	  	  		  if(myRxMessage[0] == 3){
	  	  		  maneuverObstacle();
	  	  		 // }
	  	  		//HAL_UART_Transmit(&huart2, (uint8_t *)myRxMessage, 32+2, 100);
	  	  }

	  	  	startSensor();
	 	    strtForward();
	 	 	distance=calculateDistance();
	 	 	if(distance<REQ_DIS)  //if distance is less than the REQ_DIS this means node is too close to an object needs to move
	 	 	{
	 	 		//Here we need our NRF to send out a message that all nodes need to also move
	 	 		NRF24_stopListening();      		//Turn off recieve mode so we can transmit data
	 	 		NRF24_openWritingPipe(pipeAddrs);  	//The address to write to
	 	 		generateMsg(PROX_MSG, distance);   	 	//put distance data in message array

	 	 		if(NRF24_write(myTxMessage, 32)) 	//Transmit message
	 	 		 {
	 	 		  //print to the uart to see if successfully transmitted for debugging purposes
	 	 		  sprintf(usartRead,"Transmitted MSGID: %ld, Distance =%lu - this NodeID = %ld\n\r",myTxMessage[0], myTxMessage[3], myTxMessage[1]);
	 	 		  HAL_UART_Transmit(&huart2, (uint8_t*)usartRead, strlen(usartRead), 100);
	 	 		  HAL_UART_Transmit(&huart2, (uint8_t *)"Transmitted succesfully\r\n", strlen("transmitted Succesfully\r\n"),32);
	 	 		 }

	 	 		HAL_Delay(1000);  						//wait one second
	 	 		NRF24_openReadingPipe(1, pipeAddrs);  //open reading pipeline
	 	 		NRF24_startListening();					//start listening again
	 	 		maneuverObstacle();
	 	 	}
	 	 	else
	 	 	strtForward();


	 	 	HAL_Delay(100);


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGT_BKD_Pin|CSNpin_Pin|CEpin_Pin|LFT_BKD_Pin 
                          |LFT_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|RGT_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RGT_BKD_Pin CSNpin_Pin CEpin_Pin LFT_BKD_Pin 
                           LFT_FWD_Pin */
  GPIO_InitStruct.Pin = RGT_BKD_Pin|CSNpin_Pin|CEpin_Pin|LFT_BKD_Pin 
                          |LFT_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin RGT_FWD_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|RGT_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void generateMsg(uint8_t msgID, uint32_t msgData){

	myTxMessage[0] = msgID;
	myTxMessage[1] = nodeID;
	myTxMessage[2] = 0; //until i figure out how i want to do the source thing
	myTxMessage[3] = msgData;
}

void TimerDelay(uint32_t Sec)
{
	if(Sec<2)
		Sec=2;

	TIM2->ARR=Sec-1;
	TIM2->EGR=1;
	TIM2->SR&=~1;
	TIM2->CR1|=1;

	while((TIM2->SR&0x0001)!=1);
	TIM2->SR&=~(0x0001);
}


void startSensor(void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);
  	TimerDelay(3);
	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_SET);
 	TimerDelay(10);
 	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);

}



uint32_t calculateDistance(void)
{
	uint32_t distance;
	uint32_t countTheSoundTime;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin)== GPIO_PIN_RESET);
	countTheSoundTime=0;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin)== GPIO_PIN_SET)
	{
		countTheSoundTime++;
		TimerDelay(2);
	};
	distance = (countTheSoundTime +0.0f)*2.753*speedOfSound;
	return (uint32_t)distance;
}


void strtForward(void)
{
	HAL_GPIO_WritePin(LFT_FWD_GPIO_Port, LFT_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LFT_BKD_GPIO_Port, LFT_BKD_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RGT_FWD_GPIO_Port, RGT_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RGT_BKD_GPIO_Port, RGT_BKD_Pin, GPIO_PIN_RESET);
}

void reverse(void)
{

	HAL_GPIO_WritePin(LFT_FWD_GPIO_Port, LFT_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LFT_BKD_GPIO_Port, LFT_BKD_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(RGT_FWD_GPIO_Port, RGT_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGT_BKD_GPIO_Port, RGT_BKD_Pin, GPIO_PIN_SET);
}



void turnLeft(void)
{
	HAL_GPIO_WritePin(LFT_FWD_GPIO_Port, LFT_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGT_BKD_GPIO_Port, RGT_BKD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RGT_FWD_GPIO_Port, RGT_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LFT_BKD_GPIO_Port, LFT_BKD_Pin, GPIO_PIN_SET);
}

void maneuverObstacle() {
	reverse();
	TimerDelay(4);
	turnLeft();
	TimerDelay(1.5);
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
