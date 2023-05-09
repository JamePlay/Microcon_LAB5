/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[10]; //Size of RX
uint8_t TxBuffer[100]; //Size of TX


enum screen {HomePage,LedControl,ButtonStatus,Err};
enum screen state;

uint8_t Blink_Hz = 1; //Blink time
uint8_t Led_work = 1; //On Led
uint8_t My_Button[10];
uint8_t My_Button_Before;//For Edge Check

//To check that did it show the interface?
uint8_t HomePage_do = 0;
uint8_t LedControl_do = 0;
uint8_t ButtonStatus_do = 0;
uint8_t Err_do = 0;

//ASCII Code 0(000) is Non-Function(Reset) code

uint8_t time_out = 100; // for time out

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//void UARTInterruptConfig(); // to set interrupt
void UARTDMAConfig();// set dma
void clear_screen();
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
  /* USER CODE BEGIN 2 */
  UARTDMAConfig();
  state = HomePage;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  switch (state) {
			case HomePage:
				//----------------------------------------------#Interface#----------------------------------------------
				if(HomePage_do == 0){
					HomePage_do = 1;
					LedControl_do = 0;
					ButtonStatus_do = 0;
					Err_do = 0;

					clear_screen();

					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"-------- FRA223 LAB5 UART 6444 -------- \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [0] LED Control\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [1] Button Status\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"Type the Following button in [_] to use the Function \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);

				}
				//----------------------------------------------#Action#----------------------------------------------
				if(RxBuffer[0] == 48 ){ //(0) go to led control page
					state = LedControl;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				else if(RxBuffer[0] == 49){ //(1) go to Button status page
					state = ButtonStatus;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				else if(RxBuffer[0] != 0 && RxBuffer[0] != 49 && RxBuffer[0] != 48){//Wrong Input
					state = Err;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				break;
			case LedControl:
				//----------------------------------------------#Interface#----------------------------------------------
				if(LedControl_do == 0){
					HomePage_do = 0;
					LedControl_do = 1;
					ButtonStatus_do = 0;
					Err_do = 0;

					clear_screen();
					sprintf((char*)TxBuffer,"-------- LED Control Blink Function -------- \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"---------- #LED now Blink %d Hz ------------- \r\n",Blink_Hz);
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [a] Speed Up +1 Hz\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [s] Speed Down -1 Hz\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [d] On/Off\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [x] Home Page\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"Type the Following button in [_] to use the Function \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);

				}
				//----------------------------------------------#Action#----------------------------------------------
				if(RxBuffer[0] == 120){ //(x) go to led home page
					state = HomePage;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				else if(RxBuffer[0] == 97){//(a) +1Hz
					Blink_Hz = Blink_Hz + 1;
					LedControl_do = 0; //for next frame
					RxBuffer[0] = 0;
					if(Blink_Hz > 255) Blink_Hz = 255; //Maximum
				}
				else if(RxBuffer[0] == 115){//(s) -1Hz
					Blink_Hz = Blink_Hz - 1;
					LedControl_do = 0; //for next frame
					RxBuffer[0] = 0;
					if(Blink_Hz < 1) Blink_Hz = 1; //Minimum
				}
				else if(RxBuffer[0] == 100){//(d) On/off
					if(Led_work == 0) Led_work = 1;
					else if(Led_work == 1){
						Led_work = 0;
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					}
					RxBuffer[0] = 0;
				}
				else if(RxBuffer[0] != 0 && RxBuffer[0] != 97 && RxBuffer[0] != 115 && RxBuffer[0] != 100 && RxBuffer[0] != 120){//Wrong Input
					state = Err;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				break;
			case ButtonStatus:
				//----------------------------------------------#Interface#----------------------------------------------
				if(ButtonStatus_do == 0){
					HomePage_do = 0;
					LedControl_do = 0;
					ButtonStatus_do = 1;
					Err_do = 0;

					clear_screen();
					sprintf((char*)TxBuffer,"-------- Button Status Check Function -------- \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"---------- #My Button is now %s ------------- \r\n",My_Button);
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"         [x] Home Page\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"Type the Following button in [_] to use the Function \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
				}
				//----------------------------------------------#Action#----------------------------------------------
				if(RxBuffer[0] == 120){ //(x) go to led home page
					state = HomePage;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				else if(RxBuffer[0] != 0 && RxBuffer[0] != 120){//Wrong Input
					state = Err;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1 && My_Button_Before == 0) {
					sprintf((char*)My_Button,"Unpressed");
					ButtonStatus_do = 0; //for next frame
				}
				else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0 && My_Button_Before == 1) {
					sprintf((char*)My_Button,"Pressed");
					ButtonStatus_do = 0; //for next frame
				}
				//Button
				My_Button_Before = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
				break;

				break;
			case Err:
				//----------------------------------------------#Interface#----------------------------------------------
				if(Err_do == 0){
					HomePage_do = 0;
					LedControl_do = 0;
					ButtonStatus_do = 0;
					Err_do = 1;

					clear_screen();
					sprintf((char*)TxBuffer,"************ Wrong Input! ************ \r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
					sprintf((char*)TxBuffer,"       Type [x] to Home Page\r\n");
					HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), time_out);
				}
				//----------------------------------------------#Action#----------------------------------------------
				if(RxBuffer[0] == 120){ //(x) go to led home page
					state = HomePage;
					RxBuffer[0] = 0; //If it didn't do it will not reset
				}

		}
		  //Led Blink
		  if(Led_work == 1){
		  static uint32_t timestamp = 0;
			  if(timestamp < HAL_GetTick()){
				timestamp = HAL_GetTick() + (500/Blink_Hz);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			  }
		  }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*void UARTInterruptConfig(){
	HAL_UART_Receive_IT(&huart2, RxBuffer, 1); // START Interrupt // Interrupt when collect 1 ASCII
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		RxBuffer[2] = '\0';

		//sprintf((char*)TxBuffer,"Received : %s\r\n",RxBuffer);
		//HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));

		HAL_UART_Receive_IT(&huart2, RxBuffer, 1);
	}
}*/
void UARTDMAConfig(){
	HAL_UART_Receive_DMA(&huart2, RxBuffer, 1); // START DMA
}
void clear_screen(){
	for (int i = 0; i < 20; ++i) {
		sprintf((char*)TxBuffer,"\r\n");
		HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), 10);
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
