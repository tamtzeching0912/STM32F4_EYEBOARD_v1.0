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
#include "TI_ADS1293.h"
#include "jdy16.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
////////BLE////////////
uint8_t BT_data[64];
uint8_t rx_index = 0;
uint8_t rx_data, rx_data_mcu;
bool sendCommand = false;
uint8_t app_command;
//////////EOG/////////////
bool is_ads1293_init = false;
uint8_t ads1293_val1[6], ads1293_val2;
unsigned long eog_ch1 = 0, eog_ch2 = 0;
unsigned long blink1_V[205], blink2_V[205], blink3_V[205], blink4_V[205], blink5_V[205];
unsigned long blink1_H[205], blink2_H[205], blink3_H[205], blink4_H[205], blink5_H[205];
bool sample2 = false, sample3 = false, sample4 = false, sample5 = false;
bool analyze1 = false, analyze2 = false, analyze3 = false, analyze4 = false, analyze5 = false;		//flag for blink detection
double normalized[205];
int count1 = 0, count2 = 0, count3 = 0, count4 = 0, count5 = 0;			//blink_[] index counter
bool calib = false, operate = false;											//operation mode
unsigned long LEFT, RIGHT, UP, DOWN;
unsigned long int leftThreshold, rightThreshold, upThreshold, downThreshold;
int dir = 0, region = -1;									//dir= direction counter for calibration	region=return region(final result)
int pos1[205], pos2[205], pos3[205], pos4[205], pos5[205];			//dataframe eye gaze position distribution array
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void normalization (unsigned long *dataframe);
bool blink_detection(void);		
void reset(void);					//CLEAR all dataframe
void getThreshold(void);
int direction(unsigned long, unsigned long);
int dominant_pos(int*);
////////////for printf//////////
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
	
PUTCHAR_PROTOTYPE{
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, 100);
	
	return ch;
}
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	Ads1293_Init();
	Ads1293_Config(&is_ads1293_init, hspi1);
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	JDY16_init(huart1);
	HAL_UART_Receive_IT(&huart6, &rx_data_mcu, 1);
	HAL_TIM_Base_Start_IT(&htim3);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(calib){
			if(dir == 4){
					calib = false;
					dir = 0;
					getThreshold();
					operate = true;
					uint8_t input = '6';
					HAL_UART_Transmit(&huart1, &input, 1, 100);
			}
		}
		else if(operate){
			if(analyze1){
				normalization(blink1_V);
				region = dominant_pos(pos1);
				if(blink_detection() && region!=-1){
					printf("Region %d\r\n", region);
					uint8_t input = region + '0';
					HAL_UART_Transmit(&huart1, &input, 1, 100);
					HAL_Delay(500);
					reset();
				}
				analyze1 = false;
			}
			else if(analyze2){
				normalization(blink2_V);
				region = dominant_pos(pos2);
				if(blink_detection() && region!=-1){
					printf("Region %d\r\n", region);
					uint8_t input = region + '0';
					HAL_UART_Transmit(&huart1, &input, 1, 100);
					HAL_Delay(500);
					reset();
				}
				analyze2 = false;
			}
			else if(analyze3){
				normalization(blink3_V);
				region = dominant_pos(pos3);
				if(blink_detection() && region!=-1){
					printf("Region %d\r\n", region);
					uint8_t input = region + '0';
					HAL_UART_Transmit(&huart1, &input, 1, 100);
					HAL_Delay(500);
					reset();
				}
				analyze3 = false;
			}
			else if(analyze4){
				normalization(blink4_V);
				region = dominant_pos(pos4);
				if(blink_detection() && region!=-1){
					printf("Region %d\r\n", region);
					uint8_t input = region + '0';
					HAL_UART_Transmit(&huart1, &input, 1, 100);
					HAL_Delay(500);
					reset();
				}
				analyze4 = false;
			}
			else if(analyze5){
				normalization(blink5_V);
				region = dominant_pos(pos5);
				if(blink_detection() && region!=-1){
					printf("Region %d\r\n", region);
					uint8_t input = region + '0';
					HAL_UART_Transmit(&huart1, &input, 1, 100);
					HAL_Delay(500);
					reset();
				}
				analyze5 = false;
			}
		}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1639;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nReset_Pin|Wake_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARMB_Pin */
  GPIO_InitStruct.Pin = ALARMB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALARMB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nReset_Pin Wake_Pin */
  GPIO_InitStruct.Pin = nReset_Pin|Wake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_LINK_Pin */
  GPIO_InitStruct.Pin = BT_LINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT_LINK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static int calib_count = 0;
	static unsigned long long EOG_temp = 0;
	if(htim->Instance == TIM3){
		Read_Data_Stream(ads1293_val1, 6, hspi1);
		eog_ch1 = ((unsigned long)ads1293_val1[0]<<16 | (unsigned long)ads1293_val1[1]<<8 | (unsigned long)ads1293_val1[2]);
		eog_ch2 = ((unsigned long)ads1293_val1[3]<<16 | (unsigned long)ads1293_val1[4]<<8 | (unsigned long)ads1293_val1[5]);
		//printf("%lu\r\n", eog_ch1);
		
		if(calib){
			if(dir==0 || dir==1){
				EOG_temp += eog_ch1;
			}
			else if(dir==2 || dir==3){
				EOG_temp += eog_ch2;
			}
			if(calib_count++ == 767){		//3 sec
				uint8_t input = '5';
				if(dir<3){
					HAL_UART_Transmit(&huart1, &input, 1, 100);
				}
				switch(dir++){
					case 0: 
						UP = EOG_temp/768; 
						printf("DOWN\r\n");
						break;
					case 1: 
						DOWN = EOG_temp/768; 
						printf("LEFT\r\n");
						break;
					case 2: 
						LEFT = EOG_temp/768; 
						printf("RIGHT\r\n");
						break;
					case 3: 
						RIGHT = EOG_temp/768; 
						break;
				}
				EOG_temp = 0;
				calib_count = 0;
			}
		}
		else if(operate){
			if(!sample5){
				if(count1 == 41) sample2 = true;
				if(count2 == 41) sample3 = true;
				if(count3 == 41) sample4 = true;
				if(count4 == 41) sample5 = true;
			}
			blink1_V[count1] = eog_ch1;
			blink1_H[count1] = eog_ch2;
			pos1[count1++] = direction(eog_ch1, eog_ch2);
			if(sample2){
				blink2_V[count2] = eog_ch1;
				blink2_H[count2] = eog_ch2;
				pos2[count2++] = direction(eog_ch1, eog_ch2);
			}
			if(sample3){
				blink3_V[count3] = eog_ch1;
				blink3_H[count3] = eog_ch2;
				pos3[count3++] = direction(eog_ch1, eog_ch2);
			}
			if(sample4){
				blink4_V[count4] = eog_ch1;
				blink4_H[count4] = eog_ch2;
				pos4[count4++] = direction(eog_ch1, eog_ch2);
			}
			if(sample5){
				blink5_V[count5] = eog_ch1;
				blink5_H[count5] = eog_ch2;
				pos5[count5++] = direction(eog_ch1, eog_ch2);
			}
			if(count1 == 205){
				analyze1 = true;
				count1 = 0;
			}
			else if(count2 == 205){
				analyze2 = true;
				count2 = 0;
			}
			else if(count3 == 205){
				analyze3 = true;
				count3 = 0;
			}
			else if(count4 == 205){
				analyze4 = true;
				count4 = 0;
			}
			else if(count5 == 205){
				analyze5 = true;
				count5 = 0;
			}	
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){
		if(rx_index == 0){
			for(int i=0; i<64; i++){
				BT_data[i] = 0;
			}
		}
		if(rx_data !=13){
			BT_data[rx_index++] = rx_data;
		}
		else{
			if(rx_index==2){
				if(BT_data[1] == 'c'){
					operate = false;
					printf("UP\r\n");
					calib = true;
				}
			}
			rx_index = 0;
		}
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
	if(huart->Instance == USART6){
		HAL_UART_Transmit(&huart1, &rx_data_mcu, 1, 100);
		HAL_UART_Receive_IT(&huart6, &rx_data_mcu, 1);
	}
}

void normalization (unsigned long *dataframe){
	unsigned long min, max;
	min = max = dataframe[0];
	for (int i = 0; i < 205; i++){
		if (min > dataframe[i])
			min = dataframe[i];
		if (max < dataframe[i])
			max = dataframe[i];
	}
  for (int i = 0; i < 205; i++){
		normalized[i] = ((double) dataframe[i] - (double) min) / ((double) max - (double) min);
	}
}

bool blink_detection (){
  bool down = true;
	int blinkCount = 0;
  for (int i = 0; i < 181; i++){
		double rateOfChange = (normalized[i + 24] - normalized[i]) / 25.0;
		if (down && rateOfChange <= -0.024)
			down = false;
		else if (!down && rateOfChange >= 0.024){
			down = true;
			blinkCount++;
		}
  }
	if(blinkCount == 2) return true;
	return false;
}

void reset(){
	for(int i = 0; i<205; i++){
		blink1_H[i] = blink1_V[i] = 0;
		blink2_H[i] = blink2_V[i] = 0;
		blink3_H[i] = blink3_V[i] = 0;
		blink4_H[i] = blink4_V[i] = 0;
		blink5_H[i] = blink5_V[i] = 0;
		pos1[i] = pos2[i] = pos3[i] = pos4[i] = pos5[i] = -1; 
	}
	count1 = count2 = count3 = count4 = count5 = 0;
	sample2 = sample3 = sample4 = sample5 = false;
}

void getThreshold(){
  leftThreshold = LEFT-(LEFT-RIGHT)/4;
  rightThreshold = RIGHT+(LEFT-RIGHT)/4;
  upThreshold = UP+(DOWN-UP)/4;
  downThreshold = DOWN-(DOWN-UP)/4;
}

int direction(unsigned long eog_V, unsigned long eog_H){
	//up
	if(eog_V < upThreshold && (eog_H < leftThreshold && eog_H > rightThreshold)) return 0;
	//down
	else if(eog_V > downThreshold && (eog_H < leftThreshold && eog_H > rightThreshold)) return 1;
	//left
	else if(eog_H > leftThreshold && (eog_V > upThreshold && eog_V < downThreshold)) return 2;
	//right
	else if(eog_H < rightThreshold && (eog_V > upThreshold && eog_V < downThreshold)) return 3;
	//center
	else if(eog_H < leftThreshold && eog_H > rightThreshold && eog_V > upThreshold && eog_V < downThreshold) return 4;
	return -1;
}

int dominant_pos(int* pos){
	int c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0;
	double percentage = 0.0;
	for(int i = 0; i<205; i++){
		switch(pos[i]){
			case 0: c0++; break;
			case 1: c1++; break;
			case 2: c2++; break;
			case 3: c3++; break;
			case 4: c4++; break;
		}
	}
	if((double)c0/205.0 >= 0.60) return 1;				//up
	else if((double)c1/205.0 >= 0.60) return 3;		//down
	else if((double)c2/205.0 >= 0.60) return 4;		//left
	else if((double)c3/205.0 >= 0.60) return 2;		//right
	else if((double)c4/205.0 >= 0.60) return 0;		//center
	return -1;
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
