
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Ad717x.h"
#include "AD7175_2_regs.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//메인 시스틱 변수
uint8_t SysTick_Flag;//4ms Flag(it.c 선언)
uint8_t SysTick_TaskSwitch_Flag;//Task Flag
//시리얼 변수 
uint8_t Uart_Rx_Flag;
uint8_t Uart_Tx_Buf[512];
uint8_t Uart_Rx_Buf[512];
uint8_t Uart_Rx_Timer = 0;		/* Reset Timer */
uint16_t Uart_Rx_Index = 0;		/* Reset Index */
//SPI 테스트
uint8_t com = 0x47;
uint8_t dummy = 0x00;
uint8_t RxData[2];
#define input

uint32_t Value = 0;
uint32_t Offset = 0;

typedef struct {
    float gain[];
    uint16_t datagap[];
    uint8_t index;
} CalData;
    
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM17_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f)
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart2, temp, 1, 5);
	return(ch);
}
void MainFunction(void);//4ms 메인 함수 
void UartRecv(void);//
void static Hall_Check(void);

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  printf("UART SET\r\n");
  printf("%x\r\n", AD717X_ReadRegister(AD717X_ID_REG, AD717X_DISABLE, 2));

  AD717X_Reset();

  
  printf("CHANNEL BEFORE : %x\r\n", AD717X_ReadRegister(AD717X_CHMAP0_REG, AD717X_DISABLE, 2));
  AD717X_WriteRegister(AD717X_CHMAP0_REG,
                        (AD717X_CHMAP_REG_CH_EN|AD717X_CHMAP_REG_SETUP_SEL(0x0)|AD717X_CHMAP_REG_AINPOS(0x0)|AD717X_CHMAP_REG_AINNEG(0x1)), 
                          AD717X_DISABLE, 2);
  printf("CHANNEL AFTER : %x\r\n", AD717X_ReadRegister(AD717X_CHMAP0_REG, AD717X_DISABLE, 2));
  

  printf("SETUP BEFORE : %x\r\n", AD717X_ReadRegister(AD717X_SETUPCON0_REG, AD717X_DISABLE, 2));
  AD717X_WriteRegister(AD717X_SETUPCON0_REG, 
                        (AD717X_SETUP_CONF_REG_BI_UNIPOLAR|AD717X_SETUP_CONF_REG_REFBUF_P|AD717X_SETUP_CONF_REG_REFBUF_N|
                          AD717X_SETUP_CONF_REG_AINBUF_P|AD717X_SETUP_CONF_REG_AINBUF_N|AD717X_SETUP_CONF_REG_REF_SEL(0x2)), 
                            AD717X_DISABLE, 2);
  printf("SETUP AFTER : %x\r\n", AD717X_ReadRegister(AD717X_SETUPCON0_REG, AD717X_DISABLE, 2));
  
  
  printf("FILTER BEFORE %x\r\n", AD717X_ReadRegister(AD717X_FILTCON0_REG, AD717X_DISABLE, 2));
  AD717X_WriteRegister(AD717X_FILTCON0_REG,(AD717X_FILT_CONF_REG_ENHFILT(0x0)|AD717X_FILT_CONF_REG_ORDER(0x3)|
                        AD717X_FILT_CONF_REG_ODR(0x14)), AD717X_DISABLE, 2);
  printf("FILTER AFTER %x\r\n", AD717X_ReadRegister(AD717X_FILTCON0_REG, AD717X_DISABLE, 2));


  printf("ADCMODE BEFORE %x\r\n", AD717X_ReadRegister(AD717X_ADCMODE_REG, AD717X_DISABLE, 2));
  AD717X_WriteRegister(AD717X_ADCMODE_REG,
                        (AD717X_ADCMODE_REG_REF_EN|AD717X_ADCMODE_REG_MODE(0x0)|AD717X_ADCMODE_REG_CLKSEL(0x0)), 
                          AD717X_DISABLE, 2);
  printf("ADCMODE AFTER %x\r\n", AD717X_ReadRegister(AD717X_ADCMODE_REG, AD717X_DISABLE, 2));

  Offset = AD717X_ZeroScale();

  //printf("%x\r\n", AD717X_ReadRegister(AD717X_ADCMODE_REG, AD717X_DISABLE, 2));
  //AD717X_WriteRegister(AD717X_ADCMODE_REG,(AD717X_ADCMODE_REG_MODE(0x00)), AD717X_DISABLE, 2);
  //printf("%x\r\n", AD717X_ReadRegister(AD717X_ADCMODE_REG, AD717X_DISABLE, 2));
  #if 0
  printf("%x\r\n", AD717X_ReadRegister(AD717X_CHMAP0_REG, AD717X_DISABLE, 2));
  AD717X_WriteRegister(AD717X_CHMAP0_REG, 
                        (AD717X_CHMAP_REG_CH_EN|AD717X_CHMAP_REG_SETUP_SEL(0x3)|AD717X_CHMAP_REG_AINPOS(0x1F)|AD717X_CHMAP_REG_AINNEG(0x1F)), 
                         AD717X_DISABLE, 2);
  printf("%x\r\n", AD717X_ReadRegister(AD717X_CHMAP0_REG, AD717X_DISABLE, 2));
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    AD717X_WaitForReady();
    Value =+ AD717X_ReadData();
    //printf("%d\r\n", Value);
    uint32_t Result;
    Result = Value - Offset;

    #if Result < 0
        printf("0kg\r\n");

    
    #else if 
        if(Result < 1190) 
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.083998));
        }
        else if(Result < 2088)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.095785));
        }
        else if(Result < 2829)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.106026));
        }
        else if(Result < 3452)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.115875));
        }
        else if(Result < 4045)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.123609));
        }
        else if(Result < 4612)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.130095));
        }
        else if(Result < 5182)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.135083));
        }
        else if(Result < 5745)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.139252));
        }
        else if(Result < 6283)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.143232));
        }
        else if(Result < 6832)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.14637));
        }
        else if(Result < 7386)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.14893));
        }
        else if(Result < 7924)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.151439));
        }
        else if(Result < 8444)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.153955));
        }
        else if(Result < 8991)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.155703));
        }
        else if(Result < 9520)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.157555));
        }
        else if(Result < 10069)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.158904));
        }
        else if(Result < 10597)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.160423));
        }
        else if(Result < 11133)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.161674));
        }
        else if(Result < 11672)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.162783));
        }
        else if(Result < 12210)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.1638));
        }
        else if(Result < 12764)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.164519));
        }
        else if(Result < 13291)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.165526));
        }
        else if(Result < 13843)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.166149));
        }
        else if(Result < 14385)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.166835));
        }
        else if(Result < 14925)
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.167504));
        }
        else
        {
            printf("%dkg\r\n", (int)((float)(Result) * 0.1681));
        }
    #endif
    //printf("%dkg\r\n", (int)((float)(Value-Offset) * 0.165568458));//외부
    //printf("%dkg\r\n", (int)((float)(Value-Offset) * 0.182996716));
    
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    MainFunction();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 192-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  HAL_TIM_Base_Start_IT(&htim17); 
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
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
    _Error_Handler(__FILE__, __LINE__);
  }
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#ifdef input
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
}

/* USER CODE BEGIN 4 */
void MainFunction(void)//4ms 메인 함수 
{
    if(SysTick_Flag == 1)
    {
        SysTick_Flag = 0;
        SysTick_TaskSwitch_Flag++;

        if(SysTick_TaskSwitch_Flag > 4)
        {
            SysTick_TaskSwitch_Flag = 0;
        }

        switch(SysTick_TaskSwitch_Flag)
        {
            case 0:
            {                   
                break;
            }
            case 1:
            {
                
                break;
            }
            case 2:
            {
                
                break;
            }
            case 3:
            {                
                #ifdef input
                Hall_Check();
                #endif
                break;
            }
            case 4:
            {
                UartRecv();
                break;
            }
            default :
            {
                SysTick_TaskSwitch_Flag = 0;
                break;
            }
        }
    }
}

void UartRecv(void)
{
    if(Uart_Rx_Flag == 1)
    {
        Uart_Rx_Timer++;//20ms마다 
        if(Uart_Rx_Timer > 6)//120ms 이상 수신이 없으면 버퍼 송신 
        {
            HAL_UART_Transmit(&huart2, Uart_Rx_Buf, Uart_Rx_Index, 100);

            Uart_Rx_Flag = 0;

            Uart_Rx_Timer = 0;
            Uart_Rx_Index = 0;
        }            
    }
}

void static Hall_Check(void)
{
    uint8_t tmp;
    static uint8_t Hall_Time;

    tmp = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

    if(tmp == 0)
    {      
        Hall_Time++;//20ms마다 증가
    }
    else
    {
        Hall_Time = 0;
    }

    if(Hall_Time == 5)
    {
        printf("Hall Irq\r\n");
    }
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
