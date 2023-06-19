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
#include "CLCD.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int8_t rung[5] = {0,0,0,0,0};
int16_t cho[5] = {0,0,0,0,0};
int16_t cho1Max = 1000;
int16_t choMax = 1000;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
CLCD_Name LCD1;
char LCD_send[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	CLCD_4BIT_Init(&LCD1, 16, 2, CS_GPIO_Port, CS_Pin, EN_GPIO_Port, EN_Pin,
									D4_GPIO_Port, D4_Pin, D5_GPIO_Port, D5_Pin,
									D6_GPIO_Port, D6_Pin, D7_GPIO_Port, D7_Pin);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	
	CLCD_SetCursor(&LCD1, 0, 0);
	CLCD_WriteString(&LCD1, "Start");
	HAL_Delay(500);
	CLCD_WriteString(&LCD1, ".");
	HAL_Delay(500);
	CLCD_WriteString(&LCD1, ".");
	HAL_Delay(500);
	CLCD_WriteString(&LCD1, ".");
	CLCD_Clear(&LCD1);
	CLCD_SetCursor(&LCD1, 0, 0);
	CLCD_WriteString(&LCD1, " 1  2  3  4  5");
	CLCD_SetCursor(&LCD1, 0, 1);
	CLCD_WriteString(&LCD1, " X  X  X  X  X");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			if(HAL_GPIO_ReadPin(RUNG1_GPIO_Port, RUNG1_Pin) == 0){
				if(rung[0] == 0)
					rung[0] = 1;
				if(cho[0] != 0)
				cho[0] = 0;
			}	else{
				cho[0]++;
				if(cho[0] > choMax)
					rung[0] = 0;
			}
			
			if(HAL_GPIO_ReadPin(RUNG2_GPIO_Port, RUNG2_Pin) == 0){
				if(rung[1] == 0)
					rung[1] = 1;
				if(cho[1] != 0)
				cho[1] = 0;
			}	else{
				cho[1]++;
				if(cho[1] > choMax)
					rung[1] = 0;
			}
	
			if(HAL_GPIO_ReadPin(RUNG3_GPIO_Port, RUNG3_Pin) == 0){
				if(rung[2] == 0)
					rung[2] = 1;
				if(cho[2] != 0)
				cho[2] = 0;
			}	else{
				cho[2]++;
				if(cho[2] > choMax)
					rung[2] = 0;
			}
			
			if(HAL_GPIO_ReadPin(RUNG4_GPIO_Port, RUNG4_Pin) == 0){
				if(rung[3] == 0)
					rung[3] = 1;
				if(cho[3] != 0)
				cho[3] = 0;
			}	else{
				cho[3]++;
				if(cho[3] > choMax)
					rung[3] = 0;
			}
			if(HAL_GPIO_ReadPin(RUNG5_GPIO_Port, RUNG5_Pin) == 0){
				if(rung[4] == 0)
					rung[4] = 1;
				if(cho[4] != 0)
				cho[4] = 0;
			}	else{
				cho[4]++;
				if(cho[4] > choMax)
					rung[4] = 0;
			}
		
		CLCD_SetCursor(&LCD1, 1, 1);
		if(HAL_GPIO_ReadPin(HT1_GPIO_Port, HT1_Pin) == 0 && rung[0] == 1){
			CLCD_WriteString(&LCD1, "O");
		}	else	CLCD_WriteString(&LCD1, "X");
		CLCD_SetCursor(&LCD1, 4, 1);
		if(HAL_GPIO_ReadPin(HT2_GPIO_Port, HT2_Pin) == 0 && rung[1] == 1){
			CLCD_WriteString(&LCD1, "O");
		}	else	CLCD_WriteString(&LCD1, "X");
		CLCD_SetCursor(&LCD1, 7, 1);
		if(HAL_GPIO_ReadPin(HT3_GPIO_Port, HT3_Pin) == 0 && rung[2] == 1){
			CLCD_WriteString(&LCD1, "O");
		}	else	CLCD_WriteString(&LCD1, "X");
		CLCD_SetCursor(&LCD1, 10, 1);
		if(HAL_GPIO_ReadPin(HT4_GPIO_Port, HT4_Pin) == 0 && rung[3] == 1){
			CLCD_WriteString(&LCD1, "O");
		}	else	CLCD_WriteString(&LCD1, "X");
		CLCD_SetCursor(&LCD1, 13, 1);
		if(HAL_GPIO_ReadPin(HT5_GPIO_Port, HT5_Pin) == 0 && rung[4] == 1){
			CLCD_WriteString(&LCD1, "O");
		}	else	CLCD_WriteString(&LCD1, "X");
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
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
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |EN_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RUNG1_Pin RUNG2_Pin RUNG3_Pin */
  GPIO_InitStruct.Pin = RUNG1_Pin|RUNG2_Pin|RUNG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RUNG4_Pin RUNG5_Pin HT1_Pin HT2_Pin
                           HT3_Pin HT4_Pin HT5_Pin */
  GPIO_InitStruct.Pin = RUNG4_Pin|RUNG5_Pin|HT1_Pin|HT2_Pin
                          |HT3_Pin|HT4_Pin|HT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           EN_Pin CS_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |EN_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
