/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int ustavka = 0;
uint32_t adc[1] = {0};
uint32_t abc[1] = {1578};
//char symbol;
char tmpr[5];

//uint8_t testDataToSend[8];
char User_Data[6] = {0};
int Pc_array[6] = {0};

enum {Level1, Level2, Level3};
int PwmType = 0; 

typedef struct
{
  double dState;                  // Last position input
  double iState;                  // Integrator state
  double iMax, iMin;      
  // Maximum and minimum allowable integrator state
  double    iGain,        // integral gain
            pGain,        // proportional gain
             dGain;         // derivative gain
} SPid;



// ���������� ��� �������
int a = 0; 
int b = 0; 
int c = 0;


// �������� ���������� �������� � ������������ ��������
double Uoffset = 0.9;
uint8_t K = 2;
double q = 3.3 / 4095;

// ���������� �� ������ ���������� � ������� �������� �����������
double Uout = 0;
double NowTemp = 0;
int INowTemp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
/* USER CODE BEGIN 0 */


void SendTempature(const char * str);
double UpdatePID(SPid * pid, double error, double position);
 void GetTempature();
void PwmStop(int pwm); 

/* USER CODE END 0 */

int main(void)
{
  
  /* USER CODE BEGIN 1 */
  //char temp[] = " Hello World ";
  
  abc[0] = 123;
  SPid mut; 
  mut.dState = 0;
  mut.iState = 0;
  mut.iMax = 0.2;
  mut.iMin = -0.2;
  mut.iGain = 0.1;
  mut.pGain = 10;
  mut.dGain = 100;
  
  //symbol = abc[0] + '0';
  //K++;
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  
  /* USER CODE BEGIN 2 */

    
start : 
  // wait start
  
   while (Pc_array[0] != 11) {
    HAL_Delay(1);
  }
  
  Pc_array[0] = 0;      
  //sprintf(str, "%d", number);
  // set new ustavka 
  ustavka = Pc_array[1];
  
  
nustavka : 
  
  // ��������� ������������ ���������� ��������
  if(-18 < ustavka && ustavka < 20) {
    // �������� 1 - �� ������ �� q = 1.35
    a++;
    PwmType = Level1;
    TIM4->CCR1 = 31111;
    //TIM4->CCR1 = 10000;
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  }
  
  else if(-60 < ustavka && ustavka < -18) {
    // �������� 1 - �� ������ �� q = 1.35
    // �������� 2 - �� ������ �� q = 1.35
    b++;
    PwmType = Level2;
    TIM4->CCR1 = 31111;
    TIM4->CCR2 = 31111;
    
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    
  }
  
  else {
    // �������� 1 - �� ������ �� q = 1.35
    // �������� 2 - �� ������ �� q = 1.35
    // �������� 3 - �� ������ �� q = 1.35
    c++;
    PwmType = Level3;
    TIM4->CCR1 = 31111;
    TIM4->CCR2 = 31111;
    TIM4->CCR3 = 31111;
    
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  
  /* USER CODE BEGIN 3 */
    
    // PID();
    //UpdatePID(&mut, ustavka - NowTemp, NowTemp);
    
    
    // ���������������� 1 �������, �������������  TIM4->CCR1
    
      
    
    // start ADC
    HAL_ADC_Start_IT(&hadc1);
    // ���� ���� ���������� ���������� � ����� �������� ����������� � adc
    HAL_Delay(100);     
    
    
    // ���������� ������ � ������� ����������� 
    GetTempature(); 
    SendTempature(tmpr);
    
    //SendTempature(temp);
    //HAL_Delay(500);
    
    // ��������� ������ �� ������� ����
    if(Pc_array[5] == 120)  {
      // turn off all PWM
      PwmStop(PwmType);
      Pc_array[5] = 0;
      goto start; 
    }
    
    // ��������� ������ �� ����� �������� �����������
    else if(ustavka != Pc_array[1]) {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      PwmStop(PwmType);
      ustavka = Pc_array[1];
      goto nustavka;
    }
    
    // ���������� ������
    
        
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 41999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 41999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_MspPostInit(&htim4);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

double UpdatePID(SPid * pid, double error, double position) {
    double pTerm, dTerm, iTerm;
    pTerm = pid->pGain * error;    // calculate the proportional term
    pid->iState += error;          // calculate the integral state with appropriate limiting
 
    if (pid->iState > pid->iMax) 
        pid->iState = pid->iMax;     
    else if (pid->iState < pid->iMin) 
        pid->iState = pid->iMin;
    iTerm = pid->iGain * pid->iState;    // calculate the integral term
    dTerm = pid->dGain * (position - pid->dState);
    pid->dState = position;
    return (pTerm + iTerm - dTerm);
}

void SendTempature(const char * str) {
  CDC_Transmit_FS((uint8_t*) str, strlen(str));
}

void GetTempature() {
  // ��������� �������� �������� ����������� �� �������� ���
  // ��� ������ �������� �������� ���������� �� ������ AD595
  
  // Uout = (adc[0] * q) / K - Uoffset;
  // NowTemp = -11.66 * pow(Uout, 2)+ 103.3 * Uout + 0.015;
  // INowTemp = (int) round(NowTemp);
  // sprintf(tmpr, "%d", (int) round(NowTemp));
  
  sprintf(tmpr, "%d", adc[0]);
}

void PwmStop(int pwm) {
      switch(PwmType) {
      case Level1 : {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        break;
      }
      
      case Level2 : {
       HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        break;
     }
        
      case Level3 : {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        break;
      }
      default: 
     }
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
