
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

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

//  ???? ???
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


// DC ???? ????u ????
/*
DC???? ???? 0~3 = A~D
     D
    ??
 B|  | C
    ??
     A
  [??????????]
*/
DC dc0 = {DC_A_DIR_GPIO_Port, DC_A_PWM_GPIO_Port, DC_A_DIR_Pin, DC_A_PWM_Pin , 0}; 
DC dc1 = {DC_B_DIR_GPIO_Port, DC_B_PWM_GPIO_Port, DC_B_DIR_Pin, DC_B_PWM_Pin, 0};
DC dc2 = {DC_C_DIR_GPIO_Port, DC_C_PWM_GPIO_Port, DC_C_DIR_Pin, DC_C_PWM_Pin, 0};
DC dc3 = {DC_D_DIR_GPIO_Port, DC_D_PWM_GPIO_Port, DC_D_DIR_Pin, DC_D_PWM_Pin, 0};

// BLDC ???? ????u ????  
BL bl0 = {BLDC_A_GPIO_Port, BLDC_A_Pin};  // ?? ????? 0
BL bl1 = {BLDC_B_GPIO_Port, BLDC_B_Pin};  // ??? ????? 1

//  ch0~3 = dc0~3
CH ch0 = {0, 0, 0, 0, {0, 0, 0}, 0};  // newv, oldv, width, angle, data[3], status
CH ch1 = {0, 0, 0, 0, {0, 0, 0}, 0};     
CH ch2 = {0, 0, 0, 0, {0, 0, 0}, 0};
CH ch3 = {0, 0, 0, 0, {0, 0, 0}, 0};

// pid0~3 = dc0~3
PID pid0 = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // kp, ki, kd, p, i, d, err, err_prev, de, dt, control,, kp_integer, kp_decimal, ki~, kd~
PID pid1 = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
PID pid2 = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
PID pid3 = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
               
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// ?????? ??? ???? ???? ???, Falling Edge<-> Rising Edge
static void set_polarity(TIM_TypeDef *tim,uint16_t ch,uint16_t polarity);
// 3???? ??????? ??? ??? ????? ???
float getMvAverage(__IO float *ch, __IO float value, int len);
// bldc ???? pid ??? ???
void setPidBLDC (PID pid, BL bl);
// dc ???? pid ??? ???
void setPidDC (PID *pid, CH *ch, DC *dc);

static int timer2Tick = 0;
float targetDeg = 0;
  
uint8_t aTxBuffer[] = "abcdefg";
uint8_t aRxBuffer[RXBUFFERSIZE];

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


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
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // BLDC Motor  490 Hz period  pulse width 0 ~ 2040
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);          // Bldc Motor 0
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);          // Bldc Motor 1
  HAL_TIM_Base_Start_IT(&htim1);
  
  // DC Motor 490Hz Period       pulse width 0 ~ 2040
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);          // DC Motor A
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);          // DC Motor B
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);          // DC Motor C
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);          // DC Motor D
  
  // ESC Switch 490Hz Period     pulse width 0 ~ 2040
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);          // ESC Switch
  
  // InputCaputre 0 ~ 7200 
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);           // Encoder A
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);           // Encoder B
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);           // Encoder C
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);           // Encoder D
  
  if(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    printf("set slave Receive IT Failed\n");
     _Error_Handler(__FILE__, __LINE__);
  }
  
  
  /*
   if(HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    printf("set slave Transmit IT Failed\n");
    //Error_Handler();    
  }*/
  
  
 
  /* USER CODE END 2 */

   
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  
  while (1) 
  {
  /* USER CODE END WHILE */
    
    
  /* USER CODE BEGIN 3 */
    
    
      //HAL_I2C_Slave_Receive_DMA(&hi2c1, rxData, rxLength);
      //for(int i=0;i<rxLength;i++)
       //   printf("%d  ",rxData[i]);
      //printf("\n");
      
    /*
     if (HAL_I2C_Slave_Receive_DMA(&hi2c1, rxData, rxLength) != HAL_OK)
     {
        Error_Handler();        
     }
    else
    {
    }
    
    if (HAL_I2C_Slave_Transmit_DMA(&hi2c1, txData, txLength) != HAL_OK)
     {
        Error_Handler();        
     }
    else
    {
      printf("tx OK\n");
    }    
  */
    
    
  // ch0~3?? ??????? ???¥ï? ??????? ????? ???????, ??? ??? ????? ??????? ???
    if(ch0.status == 1) 
    {
      // ?????????? ????? ???¥ï? ??????? ???, ??? ?? ????
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
    
    //??????? ??¥ì? 7200 ????? 360?? ?????? ???
    ch0.angle = ch0.width*0.05;     
    ch1.angle = ch1.width*0.05;
    ch2.angle = ch2.width*0.05;  
    ch3.angle = ch3.width*0.05;

   /* printf("ch0: %.3f\t",  ch0.angle); / printf("ch1: %.3f\t",  ch1.angle);  
    printf("ch2: %.3f\t",  ch2.angle);   printf("ch3: %.3f\t",  ch3.angle); 
    printf("\n");*/
    
    
   // pid Control
    setPidDC(&pid0, &ch3, &dc0);
    setPidDC(&pid1, &ch2, &dc1);
    setPidDC(&pid2, &ch1, &dc2);
    setPidDC(&pid3, &ch0, &dc3);
    /* DC ???? ???? ????  */
    // DC Motor Directrion, GPIO_PIN_RESET = inside, GPIO_PIN_SET = outside
   
    
    if(pid0.control > 0) 
        HAL_GPIO_WritePin(DC_A_DIR_GPIO_Port, DC_A_DIR_Pin, GPIO_PIN_SET);      
    else
        HAL_GPIO_WritePin(DC_A_DIR_GPIO_Port, DC_A_DIR_Pin, GPIO_PIN_RESET);
    if(pid1.control > 0) 
        HAL_GPIO_WritePin(DC_B_DIR_GPIO_Port, DC_B_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_B_DIR_GPIO_Port, DC_B_DIR_Pin, GPIO_PIN_RESET);    
    if(pid2.control > 0) 
        HAL_GPIO_WritePin(DC_C_DIR_GPIO_Port, DC_C_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_C_DIR_GPIO_Port, DC_C_DIR_Pin, GPIO_PIN_RESET);
    if(pid3.control > 0) 
        HAL_GPIO_WritePin(DC_D_DIR_GPIO_Port, DC_D_DIR_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DC_D_DIR_GPIO_Port, DC_D_DIR_Pin, GPIO_PIN_RESET);  
  
    // TODO abs ??? ????????. 

    TIM4 -> CCR3 = abs(dc0.setValue); // DC_A
    TIM4 -> CCR4 = abs(dc1.setValue); // DC_B
    TIM2 -> CCR3 = abs(dc2.setValue); // DC_C
    TIM2 -> CCR4 = abs(dc3.setValue); // DC_D 

    printf("A:\t"); printf("%d\t\t",dc0.setValue); printf("%f\n", ch3.angle);
    printf("B:\t"); printf("%d\t\t",dc1.setValue); printf("%f\n", ch2.angle);
    printf("C:\t"); printf("%d\t\t",dc2.setValue); printf("%f\n", ch1.angle);
    printf("D:\t"); printf("%d\t\t",dc3.setValue); printf("%f\n\n", ch0.angle);
        
  }
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_SPEEDCLOCK;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = I2C_ADDRESS;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

 HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0); 
 HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);  
 
 HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0); 
 HAL_NVIC_EnableIRQ(I2C1_ER_IRQn); 

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71; // pwm ????? 490Hz[esc??? ??????? 500hz?? ??????]?? ????? ???? Presecaler?? Period?? 71, 2040???? ???? 
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2039; // 2040
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
  sConfigOC.Pulse = 1000;
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
  htim2.Init.Prescaler = 71; // pwm ????? 490Hz[esc??? ??????? 500hz?? ??????]?? ????? ???? Presecaler?? Period?? 71, 2040???? ???? 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2039;
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
  htim3.Init.Prescaler = 8; //  ???????? ????? 1100hz?? period?? 7200???? ????????? prescaler?? 72,000,000/7200/1100= x, x?? 9???? Presecaler?? -1??? ?????? ?????? 8
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199; // ??? ??? ?????? 0.05???? ?????? 0.05 : 1 = 360 : 7200,
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
  htim4.Init.Prescaler = 71; // pwm ????? 490Hz[esc??? ??????? 500hz?? ??????]?? ????? ???? Presecaler?? Period?? 71, 2040???? ???? 
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2039;
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
 
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  pid.control = (pid.control > 2040.) ? 2040. : pid.control;
  pid.control = (pid.control < -2040.) ? -2040. : pid.control;
  pid.err_prev = pid.err;
}


void setPidDC(PID *pid, CH *ch, DC *dc)
{
  pid->err = targetDeg - ch->angle;        // ??? ?? - ???? ?? 
  pid->de = pid->err - pid->err_prev;        //  ???? ???? ????
  pid->dt = 0.001;                                  // ???? ??? 1000Hz
  pid->p = pid->err * pid->kp;                    // ???? * p????
  pid->i = pid->i + pid->err * pid->dt * pid->ki;// i ???? ????
  
  // ???????? ???? ?? ¨¨???? ??????? ???? max, min ??? ????
  pid->i = (pid->i > 100.) ? 100. : pid->i;      
  pid->i = (pid->i < -100.) ? -100. : pid->i;
  pid->d = pid->kd * (pid->de / pid->dt);      // d ???
  pid->control = pid->p + pid->i + pid->d;     // ???? ???? ????? pid ????? ????
  
  // pid ??? ???? +-2040???? ??????? ???
  pid->control = (pid->control > 2040.) ? 2040. : pid->control;           
  pid->control = (pid->control < -2040.) ? -2040. : pid->control;     
  dc->setValue = pid->control;
  pid->err_prev = pid->err;
}
                   
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    printf("hi\n");
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    printf("hi\n");
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
}


/* input capture?y? callback??? ???*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // INPUT CAPTURE ????? TIM3 ?? ???? ???????? ???
  if(htim ->Instance == TIM3)
  {
    timer2Tick ++;
      if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // TIM a???? 1?? ???
      {
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
