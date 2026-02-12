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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
#include "Compile_Data.h"
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
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t Uart_tmp[] = "Test\r\n";

uint8_t Uart_rx2_buf[rx2_buf_len] ;
uint8_t Uart_rx2_buf_tmp[3] ;
int rx2_State ;
int rx2_buf_count;
int rx2_buf_count_tmp;
int rx2_Receive_complete;

uint8_t Uart_tx_buf[tx_buf_len] ;

uint8_t RY_Status[24] ;

uint8_t DO24_Address_tmp[4] ;
uint8_t DO24_Address[1] ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////Start timer interrupt operation function///////////////////

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1){
	  HAL_IWDG_Refresh(&hiwdg);
  }

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


  /* huart2 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&huart2);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);


  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);

  DO24_Address_tmp[0] =  ~(HAL_GPIO_ReadPin(ADDR_DIP0_GPIO_Port, ADDR_DIP0_Pin)) & 0x01;
  DO24_Address_tmp[1] =  ~(HAL_GPIO_ReadPin(ADDR_DIP1_GPIO_Port, ADDR_DIP1_Pin)) & 0x01;
  DO24_Address_tmp[2] =  ~(HAL_GPIO_ReadPin(ADDR_DIP2_GPIO_Port, ADDR_DIP2_Pin)) & 0x01;
  DO24_Address_tmp[3] =  ~(HAL_GPIO_ReadPin(ADDR_DIP3_GPIO_Port, ADDR_DIP3_Pin)) & 0x01;


  DO24_Address[0] = (DO24_Address_tmp[3] << 3)|(DO24_Address_tmp[2] << 2)|(DO24_Address_tmp[1] << 1)|(DO24_Address_tmp[0] << 0);


  if(DO24_Address[0] == 0){
    PY1_ON();
    HAL_Delay(100);
    PY2_ON();
    HAL_Delay(100);
    PY3_ON();
    HAL_Delay(100);
    PY4_ON();
    HAL_Delay(100);
    PY5_ON();
    HAL_Delay(100);
    PY6_ON();
    HAL_Delay(100);
    PY7_ON();
    HAL_Delay(100);
    PY8_ON();
    HAL_Delay(100);
    PY9_ON();
    HAL_Delay(100);
    PY10_ON();
    HAL_Delay(100);
    PY11_ON();
    HAL_Delay(100);
    PY12_ON();
    HAL_Delay(100);
    PY13_ON();
    HAL_Delay(100);
    PY14_ON();
    HAL_Delay(100);
    PY15_ON();
    HAL_Delay(100);
    PY16_ON();
    HAL_Delay(100);
    PY17_ON();
    HAL_Delay(100);
    PY18_ON();
    HAL_Delay(100);
    PY19_ON();
    HAL_Delay(100);
    PY20_ON();
    HAL_Delay(100);
    PY21_ON();
    HAL_Delay(100);
    PY22_ON();
    HAL_Delay(100);
    PY23_ON();
    HAL_Delay(100);
    PY24_ON();
    HAL_Delay(100);

    HAL_Delay(1000);
    PY1_OFF();
    HAL_Delay(100);
    PY2_OFF();
    HAL_Delay(100);
    PY3_OFF();
    HAL_Delay(100);
    PY4_OFF();
    HAL_Delay(100);
    PY5_OFF();
    HAL_Delay(100);
    PY6_OFF();
    HAL_Delay(100);
    PY7_OFF();
    HAL_Delay(100);
    PY8_OFF();
    HAL_Delay(100);
    PY9_OFF();
    HAL_Delay(100);
    PY10_OFF();
    HAL_Delay(100);
    PY11_OFF();
    HAL_Delay(100);
    PY12_OFF();
    HAL_Delay(100);
    PY13_OFF();
    HAL_Delay(100);
    PY14_OFF();
    HAL_Delay(100);
    PY15_OFF();
    HAL_Delay(100);
    PY16_OFF();
    HAL_Delay(100);
    PY17_OFF();
    HAL_Delay(100);
    PY18_OFF();
    HAL_Delay(100);
    PY19_OFF();
    HAL_Delay(100);
    PY20_OFF();
    HAL_Delay(100);
    PY21_OFF();
    HAL_Delay(100);
    PY22_OFF();
    HAL_Delay(100);
    PY23_OFF();
    HAL_Delay(100);
    PY24_OFF();
  }


  for(int i=0;i<24;i++){
    RY_Status[i] = 0;
  }

  Compile_Date();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	  /* Enable the UART Data Register not empty Interrupt */
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	    if(RY_Status[0] == 1){
	      PY1_ON();
	    }
	    else if(RY_Status[0] == 0){
	      PY1_OFF();
	    }

	    if(RY_Status[1] == 1){
	      PY2_ON();
	    }
	    else if(RY_Status[1] == 0){
	      PY2_OFF();
	    }

	    if(RY_Status[2] == 1){
	      PY3_ON();
	    }
	    else if(RY_Status[2] == 0){
	      PY3_OFF();
	    }

	    if(RY_Status[3] == 1){
	      PY4_ON();
	    }
	    else if(RY_Status[3] == 0){
	      PY4_OFF();
	    }

	    if(RY_Status[4] == 1){
	      PY5_ON();
	    }
	    else if(RY_Status[4] == 0){
	      PY5_OFF();
	    }

	    if(RY_Status[5] == 1){
	      PY6_ON();
	    }
	    else if(RY_Status[5] == 0){
	      PY6_OFF();
	    }

	    if(RY_Status[6] == 1){
	      PY7_ON();
	    }
	    else if(RY_Status[6] == 0){
	      PY7_OFF();
	    }

	    if(RY_Status[7] == 1){
	      PY8_ON();
	    }
	    else if(RY_Status[7] == 0){
	      PY8_OFF();
	    }

	    if(RY_Status[8] == 1){
	      PY9_ON();
	    }
	    else if(RY_Status[8] == 0){
	      PY9_OFF();
	    }

	    if(RY_Status[9] == 1){
	      PY10_ON();
	    }
	    else if(RY_Status[9] == 0){
	      PY10_OFF();
	    }

	    if(RY_Status[10] == 1){
	      PY11_ON();
	    }
	    else if(RY_Status[10] == 0){
	      PY11_OFF();
	    }

	    if(RY_Status[11] == 1){
	      PY12_ON();
	    }
	    else if(RY_Status[11] == 0){
	      PY12_OFF();
	    }

	    if(RY_Status[12] == 1){
	      PY13_ON();
	    }
	    else if(RY_Status[12] == 0){
	      PY13_OFF();
	    }

	    if(RY_Status[13] == 1){
	      PY14_ON();
	    }
	    else if(RY_Status[13] == 0){
	      PY14_OFF();
	    }

	    if(RY_Status[14] == 1){
	      PY15_ON();
	    }
	    else if(RY_Status[14] == 0){
	      PY15_OFF();
	    }

	    if(RY_Status[15] == 1){
	      PY16_ON();
	    }
	    else if(RY_Status[15] == 0){
	      PY16_OFF();
	    }

	    if(RY_Status[16] == 1){
	      PY17_ON();
	    }
	    else if(RY_Status[16] == 0){
	      PY17_OFF();
	    }

	    if(RY_Status[17] == 1){
	      PY18_ON();
	    }
	    else if(RY_Status[17] == 0){
	      PY18_OFF();
	    }

	    if(RY_Status[18] == 1){
	      PY19_ON();
	    }
	    else if(RY_Status[18] == 0){
	      PY19_OFF();
	    }

	    if(RY_Status[19] == 1){
	      PY20_ON();
	    }
	    else if(RY_Status[19] == 0){
	      PY20_OFF();
	    }

	     if(RY_Status[20] == 1){
	      PY21_ON();
	    }
	    else if(RY_Status[20] == 0){
	      PY21_OFF();
	    }

	    if(RY_Status[21] == 1){
	      PY22_ON();
	    }
	    else if(RY_Status[21] == 0){
	      PY22_OFF();
	    }

	    if(RY_Status[22] == 1){
	      PY23_ON();
	    }
	    else if(RY_Status[22] == 0){
	      PY23_OFF();
	    }

	    if(RY_Status[23] == 1){
	      PY24_ON();
	    }
	    else if(RY_Status[23] == 0){
	      PY24_OFF();
	    }

	    if(rx2_Receive_complete == 1){
	      SW_Com();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 800;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RY5_Pin|RY6_Pin|RY7_Pin|RY8_Pin
                          |RY3_Pin|RY4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS485_DE_Pin|RS485_RE_Pin|RY21_Pin|RY22_Pin
                          |RY23_Pin|RY24_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RUN_LED_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TX_LED_Pin|RX_LED_Pin|RY1_Pin|RY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RY17_Pin|RY18_Pin|RY19_Pin|RY20_Pin
                          |RY9_Pin|RY10_Pin|RY11_Pin|RY12_Pin
                          |RY13_Pin|RY14_Pin|RY15_Pin|RY16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RY5_Pin RY6_Pin RY7_Pin RY8_Pin
                           RY3_Pin RY4_Pin */
  GPIO_InitStruct.Pin = RY5_Pin|RY6_Pin|RY7_Pin|RY8_Pin
                          |RY3_Pin|RY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DE_Pin RS485_RE_Pin RY21_Pin RY22_Pin
                           RY23_Pin RY24_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin|RS485_RE_Pin|RY21_Pin|RY22_Pin
                          |RY23_Pin|RY24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR_DIP0_Pin ADDR_DIP1_Pin ADDR_DIP2_Pin ADDR_DIP3_Pin */
  GPIO_InitStruct.Pin = ADDR_DIP0_Pin|ADDR_DIP1_Pin|ADDR_DIP2_Pin|ADDR_DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RUN_LED_Pin ERR_LED_Pin */
  GPIO_InitStruct.Pin = RUN_LED_Pin|ERR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_LED_Pin RX_LED_Pin RY1_Pin RY2_Pin */
  GPIO_InitStruct.Pin = TX_LED_Pin|RX_LED_Pin|RY1_Pin|RY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RY17_Pin RY18_Pin RY19_Pin RY20_Pin
                           RY9_Pin RY10_Pin RY11_Pin RY12_Pin
                           RY13_Pin RY14_Pin RY15_Pin RY16_Pin */
  GPIO_InitStruct.Pin = RY17_Pin|RY18_Pin|RY19_Pin|RY20_Pin
                          |RY9_Pin|RY10_Pin|RY11_Pin|RY12_Pin
                          |RY13_Pin|RY14_Pin|RY15_Pin|RY16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


int Uart2_crc = 0;
int Uart_crc = 0;

void SW_Com(void){

  // CRC 체크

  int i;

  // 전송할 중계기 정보를 확인한다.

  Uart2_crc = 0;

  for(i = 0; i <rx2_buf_count_tmp - 5 ; i++){
    Uart2_crc = Uart2_crc ^ Uart_rx2_buf[i+2];
  }

  if(Uart2_crc == Uart_rx2_buf[rx2_buf_count_tmp - 3]){
    //0x51 , 'Q' ,중계기 정보 요청 (UI에서 설정한 값만)

    if(Uart_rx2_buf[4] == 0x44){        // DO24 board: '0x44'
      if(Uart_rx2_buf[5] == DO24_Address[0]){      // Address 01
        if(Uart_rx2_buf[3] == 0x51){

          Uart_tx_buf[0] = 0x53;    //S
          Uart_tx_buf[1] = 0x54;    //T
          Uart_tx_buf[2] = 0x44;    //D
          Uart_tx_buf[3] = 0x52;    //R
          Uart_tx_buf[4] = DO24_Address[0];    //address
          Uart_tx_buf[5] = 0x0A;    //version 10 -> 1.0
          Uart_tx_buf[6] = 0x00;    //Relay 정보 1 ~8
          Uart_tx_buf[7] = 0x00;    //Relay 정보 9 ~16
          Uart_tx_buf[8] = 0x00;    //Relay 정보 17 ~24
          Uart_tx_buf[9] = 0x00;   //dummy1
          Uart_tx_buf[10] = 0x00;   //dummy2
          Uart_tx_buf[11] = 0x00;   //CRC
          Uart_tx_buf[12] = 0x45;   //E
          Uart_tx_buf[13] = 0x44;   //D

          Uart_crc = 0;

          for(i=0 ; i<8 ; i++){
            Uart_tx_buf[6] = 0;
            Uart_tx_buf[6] = (RY_Status[0] << 7) | (RY_Status[1] << 6) | (RY_Status[2] << 5) | (RY_Status[3] << 4) |
                             (RY_Status[4] << 3) | (RY_Status[5] << 2) | (RY_Status[6] << 1) | (RY_Status[7] << 0);
            Uart_tx_buf[7] = 0;
            Uart_tx_buf[7] = (RY_Status[0+8] << 7) | (RY_Status[1+8] << 6) | (RY_Status[2+8] << 5) | (RY_Status[3+8] << 4) |
                             (RY_Status[4+8] << 3) | (RY_Status[5+8] << 2) | (RY_Status[6+8] << 1) | (RY_Status[7+8] << 0);
            Uart_tx_buf[8] = 0;
            Uart_tx_buf[8] = (RY_Status[0+8+8] << 7) | (RY_Status[1+8+8] << 6) | (RY_Status[2+8+8] << 5) | (RY_Status[3+8+8] << 4) |
                             (RY_Status[4+8+8] << 3) | (RY_Status[5+8+8] << 2) | (RY_Status[6+8+8] << 1) | (RY_Status[7+8+8] << 0);

          }
          for(i = 0; i <tx_buf_len - 5 ; i++){
            Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
          }
          Uart_tx_buf[11] = Uart_crc;

          HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
          HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

          HAL_Delay(1);

          if(HAL_UART_Transmit(&huart2, Uart_tx_buf, sizeof(Uart_tx_buf), 1000)!= HAL_OK)
              {
                Error_Handler();
              }

          HAL_Delay(1);

          HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

        }
        else if(Uart_rx2_buf[3] == 0x53){
          for(i=0;i<8;i++){
            RY_Status[i] = (Uart_rx2_buf[6] >> (7 - i)) & 0x01;
            RY_Status[i+8] = (Uart_rx2_buf[7] >> (7 - i)) & 0x01;
            RY_Status[i+8+8] = (Uart_rx2_buf[8] >> (7 - i)) & 0x01;
          }
        }
        else if(Uart_rx2_buf[3] == 0x56){
        	SUB_Com_SW_V();
        }

      }
    }
  }

  rx2_Receive_complete = 0;

}


void PY1_ON(void){
	HAL_GPIO_WritePin(RY1_GPIO_Port,RY1_Pin,GPIO_PIN_SET);
}
void PY1_OFF(void){
	HAL_GPIO_WritePin(RY1_GPIO_Port,RY1_Pin,GPIO_PIN_RESET);
}
void PY2_ON(void){
	HAL_GPIO_WritePin(RY2_GPIO_Port,RY2_Pin,GPIO_PIN_SET);
}
void PY2_OFF(void){
	HAL_GPIO_WritePin(RY2_GPIO_Port,RY2_Pin,GPIO_PIN_RESET);
}
void PY3_ON(void){
	HAL_GPIO_WritePin(RY3_GPIO_Port,RY3_Pin,GPIO_PIN_SET);
}
void PY3_OFF(void){
	HAL_GPIO_WritePin(RY3_GPIO_Port,RY3_Pin,GPIO_PIN_RESET);
}
void PY4_ON(void){
	HAL_GPIO_WritePin(RY4_GPIO_Port,RY4_Pin,GPIO_PIN_SET);
}
void PY4_OFF(void){
	HAL_GPIO_WritePin(RY4_GPIO_Port,RY4_Pin,GPIO_PIN_RESET);
}
void PY5_ON(void){
	HAL_GPIO_WritePin(RY5_GPIO_Port,RY5_Pin,GPIO_PIN_SET);
}
void PY5_OFF(void){
	HAL_GPIO_WritePin(RY5_GPIO_Port,RY5_Pin,GPIO_PIN_RESET);
}
void PY6_ON(void){
	HAL_GPIO_WritePin(RY6_GPIO_Port,RY6_Pin,GPIO_PIN_SET);
}
void PY6_OFF(void){
	HAL_GPIO_WritePin(RY6_GPIO_Port,RY6_Pin,GPIO_PIN_RESET);
}void PY7_ON(void){
	HAL_GPIO_WritePin(RY7_GPIO_Port,RY7_Pin,GPIO_PIN_SET);
}
void PY7_OFF(void){
	HAL_GPIO_WritePin(RY7_GPIO_Port,RY7_Pin,GPIO_PIN_RESET);
}
void PY8_ON(void){
	HAL_GPIO_WritePin(RY8_GPIO_Port,RY8_Pin,GPIO_PIN_SET);
}
void PY8_OFF(void){
	HAL_GPIO_WritePin(RY8_GPIO_Port,RY8_Pin,GPIO_PIN_RESET);
}
void PY9_ON(void){
	HAL_GPIO_WritePin(RY9_GPIO_Port,RY9_Pin,GPIO_PIN_SET);
}
void PY9_OFF(void){
	HAL_GPIO_WritePin(RY9_GPIO_Port,RY9_Pin,GPIO_PIN_RESET);
}
void PY10_ON(void){
	HAL_GPIO_WritePin(RY10_GPIO_Port,RY10_Pin,GPIO_PIN_SET);
}
void PY10_OFF(void){
	HAL_GPIO_WritePin(RY10_GPIO_Port,RY10_Pin,GPIO_PIN_RESET);
}
void PY11_ON(void){
	HAL_GPIO_WritePin(RY11_GPIO_Port,RY11_Pin,GPIO_PIN_SET);
}
void PY11_OFF(void){
	HAL_GPIO_WritePin(RY11_GPIO_Port,RY11_Pin,GPIO_PIN_RESET);
}
void PY12_ON(void){
	HAL_GPIO_WritePin(RY12_GPIO_Port,RY12_Pin,GPIO_PIN_SET);
}
void PY12_OFF(void){
	HAL_GPIO_WritePin(RY12_GPIO_Port,RY12_Pin,GPIO_PIN_RESET);
}
void PY13_ON(void){
	HAL_GPIO_WritePin(RY13_GPIO_Port,RY13_Pin,GPIO_PIN_SET);
}
void PY13_OFF(void){
	HAL_GPIO_WritePin(RY13_GPIO_Port,RY13_Pin,GPIO_PIN_RESET);
}
void PY14_ON(void){
	HAL_GPIO_WritePin(RY14_GPIO_Port,RY14_Pin,GPIO_PIN_SET);
}
void PY14_OFF(void){
	HAL_GPIO_WritePin(RY14_GPIO_Port,RY14_Pin,GPIO_PIN_RESET);
}
void PY15_ON(void){
	HAL_GPIO_WritePin(RY15_GPIO_Port,RY15_Pin,GPIO_PIN_SET);
}
void PY15_OFF(void){
	HAL_GPIO_WritePin(RY15_GPIO_Port,RY15_Pin,GPIO_PIN_RESET);
}
void PY16_ON(void){
	HAL_GPIO_WritePin(RY16_GPIO_Port,RY16_Pin,GPIO_PIN_SET);
}
void PY16_OFF(void){
	HAL_GPIO_WritePin(RY16_GPIO_Port,RY16_Pin,GPIO_PIN_RESET);
}
void PY17_ON(void){
	HAL_GPIO_WritePin(RY17_GPIO_Port,RY17_Pin,GPIO_PIN_SET);
}
void PY17_OFF(void){
	HAL_GPIO_WritePin(RY17_GPIO_Port,RY17_Pin,GPIO_PIN_RESET);
}
void PY18_ON(void){
	HAL_GPIO_WritePin(RY18_GPIO_Port,RY18_Pin,GPIO_PIN_SET);
}
void PY18_OFF(void){
	HAL_GPIO_WritePin(RY18_GPIO_Port,RY18_Pin,GPIO_PIN_RESET);
}
void PY19_ON(void){
	HAL_GPIO_WritePin(RY19_GPIO_Port,RY19_Pin,GPIO_PIN_SET);
}
void PY19_OFF(void){
	HAL_GPIO_WritePin(RY19_GPIO_Port,RY19_Pin,GPIO_PIN_RESET);
}
void PY20_ON(void){
	HAL_GPIO_WritePin(RY20_GPIO_Port,RY20_Pin,GPIO_PIN_SET);
}
void PY20_OFF(void){
	HAL_GPIO_WritePin(RY20_GPIO_Port,RY20_Pin,GPIO_PIN_RESET);
}
void PY21_ON(void){
	HAL_GPIO_WritePin(RY21_GPIO_Port,RY21_Pin,GPIO_PIN_SET);
}
void PY21_OFF(void){
	HAL_GPIO_WritePin(RY21_GPIO_Port,RY21_Pin,GPIO_PIN_RESET);
}
void PY22_ON(void){
	HAL_GPIO_WritePin(RY22_GPIO_Port,RY22_Pin,GPIO_PIN_SET);
}
void PY22_OFF(void){
	HAL_GPIO_WritePin(RY22_GPIO_Port,RY22_Pin,GPIO_PIN_RESET);
}
void PY23_ON(void){
	HAL_GPIO_WritePin(RY23_GPIO_Port,RY23_Pin,GPIO_PIN_SET);
}
void PY23_OFF(void){
	HAL_GPIO_WritePin(RY23_GPIO_Port,RY23_Pin,GPIO_PIN_RESET);
}
void PY24_ON(void){
	HAL_GPIO_WritePin(RY24_GPIO_Port,RY24_Pin,GPIO_PIN_SET);
}
void PY24_OFF(void){
	HAL_GPIO_WritePin(RY24_GPIO_Port,RY24_Pin,GPIO_PIN_RESET);
}


#define Sub_Do_length      15

void SUB_Com_SW_V(void){

	uint8_t Uart_crc;

    Uart_tx_buf[0] = 0x53;    //S
    Uart_tx_buf[1] = 0x54;    //T
    Uart_tx_buf[2] = 0x53;    //S
    Uart_tx_buf[3] = 0x76;    //v
    Uart_tx_buf[4] = 0x44;    //DO
    Uart_tx_buf[5] = DO24_Address[0];    //address
    Uart_tx_buf[6] = F_Version_Year;
    Uart_tx_buf[7] = F_Version_Month;
    Uart_tx_buf[8] = F_Version_Day;
    Uart_tx_buf[9] = F_Version_Hour;
    Uart_tx_buf[10] = F_Version_Min;
    Uart_tx_buf[11] = F_Version_Sec;
    Uart_tx_buf[12] = 0x00;   //CRC
    Uart_tx_buf[13] = 0x45;   //E
    Uart_tx_buf[14] = 0x44;   //D


    for(int i = 0; i <Sub_Do_length - 5 ; i++){
      Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
    }
    Uart_tx_buf[Sub_Do_length-3] = Uart_crc;


    HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

    HAL_Delay(1);
    if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Sub_Do_length, 1000)!= HAL_OK)
        {
          Error_Handler();
        }

    HAL_Delay(1);

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

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
