
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "rs485.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define PIDGAINBUFFER 39

/* Private variables ---------------------------------------------------------*/


// 
/*
DC Pin Number 0~3 = A~D
     D
    
 C |  | C
    
     B
  []
*/
DC dc0 = {DC_A_DIR_GPIO_Port, DC_A_PWM_GPIO_Port, DC_A_DIR_Pin, DC_A_PWM_Pin , 0}; 
DC dc1 = {DC_B_DIR_GPIO_Port, DC_B_PWM_GPIO_Port, DC_B_DIR_Pin, DC_B_PWM_Pin, 0};
DC dc2 = {DC_C_DIR_GPIO_Port, DC_C_PWM_GPIO_Port, DC_C_DIR_Pin, DC_C_PWM_Pin, 0};
DC dc3 = {DC_D_DIR_GPIO_Port, DC_D_PWM_GPIO_Port, DC_D_DIR_Pin, DC_D_PWM_Pin, 0};

// BLDC 
BL bl0 = {BLDC_A_GPIO_Port, BLDC_A_Pin}; 
BL bl1 = {BLDC_B_GPIO_Port, BLDC_B_Pin}; 

//  ch0~3 = dc0~3
CH ch0 = {0, 0, 0, 0, {0, 0, 0}, 0};  // newv, oldv, width, angle, data[3], status
CH ch1 = {0, 0, 0, 0, {0, 0, 0}, 0};     
CH ch2 = {0, 0, 0, 0, {0, 0, 0}, 0};
CH ch3 = {0, 0, 0, 0, {0, 0, 0}, 0};

// pid0~3 = dc0~3
PID pid0 = {0.5, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // kp, ki, kd, p, i, d, err, err_prev, de, dt, control,, kp_integer, kp_decimal, ki~, kd~
PID pid1 = {0.5, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
PID pid2 = {0.5, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
PID pid3 = {0.5, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//  Falling Edge<-> Rising Edge
static void set_polarity(TIM_TypeDef *tim,uint16_t ch,uint16_t polarity);
float getMvAverage(__IO float *ch, __IO float value, int len);
void setPidBLDC (PID pid, BL bl);
void setPidDC (PID *pid, CH *ch, DC *dc);
void sepPidGain(PID *pid, int bufCount);
void sumPidGain(PID *pid); 
void entPidGain();
float map(float x, float in_min, float in_max, float out_min, float out_max);
void USART_ClearITPendingBit(UART_HandleTypeDef* USARTx, uint16_t USART_IT);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static int timer2Tick = 0;
float targetDeg = 0.;
int8_t startPidGain = 1;
uint8_t aTxBuffer[] = "";
uint8_t ReadyToFlyBuf[READY_TO_FLY_BYTE];
uint8_t setPidGainBuf[64];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // BLDC Motor  490 Hz period  pulse width 0 ~ 2040
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);             // Bldc Motor 0
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);             // Bldc Motor 1
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);           // Bldc Motor 1
  HAL_TIM_Base_Start_IT(&htim1);
  
  // DC Motor 490Hz Period   pulse width 0 ~ 1020
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);             // DC Motor A
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);             // DC Motor B
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);             // DC Motor C
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);             // DC Motor D
  
  // ESC Switch 490Hz Period     pulse width 0 ~ 2040
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);             // ESC Switch
  
  // InputCaputre 0 ~ 7200 
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);           // Encoder A
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);           // Encoder B
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);           // Encoder C
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);           // Encoder D
  
    //USART_ClearITPendingBit(&huart3, UART_IT_TC);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1) 
  {
    
    HAL_UART_Receive_IT(&huart3, (uint8_t*)ReadyToFlyBuf, READY_TO_FLY_BYTE);
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    
    
    //printf("%d\r\n",ReadyToFlyBuf[0]);
    entPidGain();
    printf("%.3f %.3f %.3f %.3f \r\n", pid0.kd, pid1.kd, pid2.kd, pid3.kd);
   //printf("%d %d %d \r\n", pid0.kp_integer, pid0.kp_decimalL, pid0.kp_decimalH);
    
    if(ch0.status == 1) 
    {
        getMvAverage(ch0.data, ch0.width, 3); 
        ch0.status = 0;
    }
    if(ch1.status == 1)
    {
        getMvAverage(ch1.data, ch1.width, 3);
        ch1.status = 0;
    }
    
    if(ch2.status == 1)
    {
        getMvAverage(ch2.data, ch2.width, 3); 
        ch2.status = 0;
    }
    if(ch3.status == 1)
    {
        getMvAverage(ch3.data, ch3.width, 3);
        ch3.status = 0;
    }
    
    
    
    ch0.angle = ch0.width*0.05;     
    ch1.angle = ch1.width*0.05;
    ch2.angle = ch2.width*0.05;  
    ch3.angle = ch3.width*0.05;
    
    
    //targetDeg = 300.;
    
    // pid Control
    setPidDC(&pid0, &ch3, &dc0);
    setPidDC(&pid1, &ch2, &dc1);
    setPidDC(&pid2, &ch1, &dc2);
    setPidDC(&pid3, &ch0, &dc3);
       
    /* DC  */
    // DC Motor Directrion, GPIO_PIN_RESET = inside, GPIO_PIN_SET = outside

    if(dc0.setValue > 0)
        HAL_GPIO_WritePin(DC_A_DIR_GPIO_Port, DC_A_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_A_DIR_GPIO_Port, DC_A_DIR_Pin, GPIO_PIN_RESET);
    if(dc1.setValue > 0) 
        HAL_GPIO_WritePin(DC_B_DIR_GPIO_Port, DC_B_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_B_DIR_GPIO_Port, DC_B_DIR_Pin, GPIO_PIN_RESET);    
    if(dc2.setValue > 0) 
        HAL_GPIO_WritePin(DC_C_DIR_GPIO_Port, DC_C_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_C_DIR_GPIO_Port, DC_C_DIR_Pin, GPIO_PIN_RESET);
    if(dc3.setValue > 0) 
        HAL_GPIO_WritePin(DC_D_DIR_GPIO_Port, DC_D_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_D_DIR_GPIO_Port, DC_D_DIR_Pin, GPIO_PIN_RESET);  
   /* 
    TIM4 -> CCR3 = abs(dc0.setValue); // DC_A
    TIM4 -> CCR4 = abs(dc1.setValue); // DC_B
    TIM2 -> CCR4 = abs(dc2.setValue); // DC_C
    TIM2 -> CCR3 = abs(dc3.setValue); // DC_D 
   
    printf("%d\t",  dc0.setValue);   printf("%d\t",  dc1.setValue);  
    printf("%d\t",  dc2.setValue);   printf("%d\t",  dc3.setValue); 
    printf("\n");
    
    printf("A:\t"); printf("%d\t",dc0.setValue); printf("%.3f\n", ch3.angle);
    printf("B:\t"); printf("%d\t",dc1.setValue); printf("%.3f\n", ch2.angle);
    printf("C:\t"); printf("%d\t",dc2.setValue); printf("%.3f\n", ch1.angle);
    printf("D:\t"); printf("%d\t",dc3.setValue); printf("%.3f\n\n", ch0.angle); 
   */ 
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2040;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2040;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2040;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Gimbal_DIR_Pin|DC_E_DIR_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_A_DIR_Pin|DC_B_DIR_Pin|DC_C_DIR_Pin|DC_D_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Gimbal_DIR_Pin DC_E_DIR_Pin LED_Pin */
  GPIO_InitStruct.Pin = Gimbal_DIR_Pin|DC_E_DIR_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_A_DIR_Pin DC_B_DIR_Pin DC_C_DIR_Pin DC_D_DIR_Pin */
  GPIO_InitStruct.Pin = DC_A_DIR_Pin|DC_B_DIR_Pin|DC_C_DIR_Pin|DC_D_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  
  
  
  
}

/* USER CODE BEGIN 4 */


// TODO: targetDeg ??? rpi???? ???? YAW???? ?????????.
void setPidBLDC(PID pid, BL bl)
{
  pid.err; //= targetDeg - ch.angle;
  pid.de = pid.err - pid.err_prev;
  pid.dt = 0.001;
  pid.p = pid.err * pid.kp;
  pid.i = pid.i + pid.err * pid.dt * pid.ki;
  pid.i = (pid.i > 10.) ? 10. : pid.i;
  pid.i = (pid.i < -10.) ? -10. : pid.i;
  pid.d = pid.kd * (pid.de / pid.dt);
  pid.control = pid.p + pid.i + pid.d;
  pid.control = (pid.control > 1020.) ? 1020. : pid.control;
  pid.control = (pid.control < -1020.) ? -1020. : pid.control;
  pid.err_prev = pid.err;
}


void setPidDC(PID *pid, CH *ch, DC *dc)
{
  
  float temp = targetDeg - ch->angle;
  if(temp < -180) {
      pid->err = 360 + temp;
  }
  else if ( temp >= 180) {
      pid->err = -360 + temp;
  }
  else {
      pid->err = temp;
  }
  
  pid->de = pid->err - pid->err_prev;        
  pid->dt = 0.001;                                 
  pid->p = pid->err * pid->kp;                    
  pid->i += pid->err * pid->dt * pid->ki;
  pid->i = (pid->i > 20.) ? 20. : pid->i;      
  pid->i = (pid->i < -20.) ? -20. : pid->i;
  pid->d = pid->kd * (pid->de / pid->dt);      
  pid->control = pid->p + pid->i + pid->d;       
 
  //printf("p: %.2f\t", pid->p); printf("i: %.2f\t", pid->i); printf("d: %.2f\t", pid->d);  printf("pid: %.3f\n", pid->control);
  pid->control = constrain(pid->control,-510,510);
  dc->setValue = pid->control;
  pid->err_prev = pid->err;
}

void sepPidGain(PID *pid, int bufCount)
{
  pid->kp_integer  = ReadyToFlyBuf[bufCount+1];  
  pid->kp_decimalL = ReadyToFlyBuf[bufCount+2];  
  pid->kp_decimalH = ReadyToFlyBuf[bufCount+3];  
  pid->ki_integer  = ReadyToFlyBuf[bufCount+4];   
  pid->ki_decimalL = ReadyToFlyBuf[bufCount+5];  
  pid->ki_decimalH = ReadyToFlyBuf[bufCount+6];  
  pid->kd_integer  = ReadyToFlyBuf[bufCount+7];  
  pid->kd_decimalL = ReadyToFlyBuf[bufCount+8];  
  pid->kd_decimalH = ReadyToFlyBuf[bufCount+9];  
}

void sumPidGain(PID *pid)
{
  float temp;
  pid->kp =  pid->kp_integer;
  //temp = (float)pid->kp_decimalH << 8;
  
  
  pid->kp += (float)((pid->kp_decimalH << 8) | (pid->kp_decimalL)) * 0.001;
  
  pid->ki =  pid->ki_integer;
  pid->ki += (float)((pid->ki_decimalH << 8) | (pid->ki_decimalL)) * 0.001;
  
  pid->kd =  pid->kd_integer;
  pid->kd += (float)((pid->kd_decimalH << 8) | (pid->kd_decimalL)) * 0.001;
}

void entPidGain()
{
    int sum = 0;
    for (int i=0;i<READY_TO_FLY_BYTE-1;i++)
    {
      sum += ReadyToFlyBuf[i];
    }
    sum = (sum & 0xff) + (sum >> 8);
    
    sum = ~sum;
    
    // sum == 1 ok, sum != 1 not ok
    
    ReadyToFlyBuf[READY_TO_FLY_BYTE-1];
    
    sepPidGain(&pid0, -1);
    sepPidGain(&pid1, 8);
    sepPidGain(&pid2, 17);
    sepPidGain(&pid3, 26);
    
    sumPidGain(&pid0);
    sumPidGain(&pid1);
    sumPidGain(&pid2);
    sumPidGain(&pid3);
  
}
        
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  USART_ClearITPendingBit(&huart3, UART_IT_TC);
  /* consume the received character */
/*
  if(huart == &huart3)
  {
    
    readCallback++;
    
    data.RxBuf[data.rx_point_head++] = myChar;
    data.rx_point_head %= RBUF_SIZE;         
  
    reqReceived = 1;
    
  }  */
  
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

  //UART_RxAgain(huart);
}




//USART_ClearITPendingBit(USARTx, USART_IT_TC)

void USART_ClearITPendingBit(UART_HandleTypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT));
  /* The CTS interrupt is not available for UART4 and UART5 */
  if (USART_IT == UART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
  
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->Instance->SR = (uint16_t)~itmask;
}
  


/* input capture?y? callback??? ???*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // INPUT CAPTURE ????? TIM3 ?? ???? ???????? ???
  if(htim ->Instance == TIM3)
  {
      if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // TIM a???? 1?? ???
      {
            timer2Tick ++;
            ch0.newv = htim ->Instance->CCR1;                 // a??0 newv?? ???? ?©£? ????
            if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6))    //  RISING EDGE???? ???? ???? ???
            {
                ch0.oldv = ch0.newv;                                   //  ????? FALLING EDGE?? ???? ??? ?? ????? ???????? ??????? old?? ????
                set_polarity(TIM3,1,2);                                 //   ?????? ?????? RISING EDGE???? FALLING EDGE?? ???
            }
            else
            {      
                set_polarity(TIM3,1,0);                                 // ?????? ?????? FALLING EDGE???? RISING EDGE?? ???
                // falling ???? ???? rising???? ???? ???? ?????? ????? ??? [?????¡À¥ï???? ???]
                ch0.width  = (ch0.newv > ch0.oldv)? (ch0.newv - ch0.oldv) : (7200 + ch0.newv - ch0.oldv); 
                ch0.status = 1;  // ???¥ï? ??? ?? ??????? ????
            }
      }
      else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
      {
            ch1.newv = htim ->Instance->CCR2;
            if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7))
            {
                ch1.oldv = ch1.newv;
                set_polarity(TIM3,2,2);
            }
            else
            {
                set_polarity(TIM3,2,0);
                ch1.width  = (ch1.newv > ch1.oldv)? (ch1.newv - ch1.oldv) : (7200 + ch1.newv - ch1.oldv);
                ch1.status = 1;
            }      
      }
      else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
      {
            ch2.newv = htim ->Instance->CCR3;
            if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8))
            {
                ch2.oldv = ch2.newv;
                set_polarity(TIM3,3,2);
            }
            else
            {
                set_polarity(TIM3,3,0);
                ch2.width  = (ch2.newv > ch2.oldv)? (ch2.newv - ch2.oldv) : (7200 + ch2.newv - ch2.oldv);
                ch2.status = 1;
            }      
      }
    
       else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
      {
            ch3.newv = htim ->Instance->CCR4;
            if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9))
            {
                ch3.oldv = ch3.newv;
                set_polarity(TIM3,4,2);
            }
            else
            {
                set_polarity(TIM3,4,0);
                ch3.width  = (ch3.newv > ch3.oldv)? (ch3.newv - ch3.oldv) : (7200 + ch3.newv - ch3.oldv);
                ch3.status = 1;
            }      
      }
  }
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ?????? ??? ???? ???? ???, TIM 1a?¥ç? RISING, FALLING?? ??????? ???? ???
void set_polarity(TIM_TypeDef *tim,uint16_t ch,uint16_t polarity)
{ // ch(1,2,3,4), polarity 0: rise,high, 2: fall,low
  uint16_t c = TIM_CCER_CC1P << ((ch-1)*4);
  if( polarity == 0 )           // RISING???? ???
      tim->CCER &= ~c;
  else                            // FALLING???? ???
      tim->CCER |= c;
}

// 3???? ??????? ??? ??? ???? ???
float getMvAverage(__IO float *ch, __IO float value, int len)
{
    float sum = 0;
    
    //  ??????? ??? ?????, ex ?ò÷[1]?? ??? ??????? ?ò÷ [0]???? ???, 
    for(int i=0;i<len-1;i++)
      ch[i] = ch[i]+1;
       
    // ?ò÷ ???????? ???¥ï? ?????? ????
    ch[len-1] = value; 
    
    //  ??? ???? ??????? ???? ??????? ???
    for(int i=0;i<len;i++)
      sum += ch[i];
    
    return sum / (float)len; // ??? ??????? ?????? ????? ???? ???
    
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
