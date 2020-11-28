
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "nrf24l01P.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
nRF24L01P myNRF;
const uint8_t RXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t TXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

uint8_t TXData[32] = "D=result_of_mpu6050";
uint8_t RXBuffer[32], TXBuffer[32];

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define accel_bound 8192
#define myMPU MPU6050_DEFAULT_ADDRESS

void MPU6050_Write(uint8_t Address, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, myMPU, Address, 1, (uint8_t *)&data, 1, 10);
}
 
void MPU6050_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data){
    uint8_t tmp = 0;
    HAL_I2C_Mem_Read(&hi2c1, myMPU, Address, 1, (uint8_t *)&tmp, 1, 10);
    uint8_t mask = 0;
    switch(length){
        case 1: mask = 0x01; break;
        case 2: mask = 0x03; break;
        case 3: mask = 0x07; break;
        case 4: mask = 0x0F; break;
        case 5: mask = 0x1F; break;
        case 6: mask = 0x3F; break;
        case 7: mask = 0x7F; break;
        case 8: mask = 0xFF; break;
    }
    tmp &= ~(mask << bitStart);
    tmp |= (data << bitStart);
    HAL_I2C_Mem_Write(&hi2c1, myMPU, Address, 1, (uint8_t *)&tmp, 1, 10);
}
 
uint8_t MPU6050_Read(uint8_t Address){
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, myMPU, Address, 1, (uint8_t *)&data, 1, 10);
    return data;
}
 
void init_MPU6050(void){
    uint8_t temp = MPU6050_Read(MPU6050_RA_WHO_AM_I);
    /*
    printf("Who am I = 0x%02X\r\n", temp);
    printf("MPU6050 Initialize..... \r\n");
    printf("------------------------\r\n");
    */
    HAL_Delay(100);
    MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0x01, DISABLE); // SetSleepModeStatus
    HAL_Delay(10);
    MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);      // SetClockSource
    MPU6050_Write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);   // SetFullScaleGyroRange
    MPU6050_Write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2); // SetFullScaleAccelRange
}
 
void read_MPU6050_data(void){
    Accel_X = (MPU6050_Read(MPU6050_RA_ACCEL_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_XOUT_L);
    Accel_Y = (MPU6050_Read(MPU6050_RA_ACCEL_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_YOUT_L);
    Accel_Z = (MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
    Gyro_X  = (MPU6050_Read(MPU6050_RA_GYRO_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_XOUT_L);
    Gyro_Y  = (MPU6050_Read(MPU6050_RA_GYRO_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_YOUT_L);
    Gyro_Z  = (MPU6050_Read(MPU6050_RA_GYRO_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_ZOUT_L);
}
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  init_MPU6050();
  
  myNRF.hspi = &hspi1;
  myNRF.CRC_Width = nRF_CRC_WIDTH_BYTE;
  myNRF.ADDR_Width = nRF_ADDR_WIDTH_5;
  myNRF.Data_Rate = nRF_DATA_RATE_2MBPS;
  myNRF.TX_Power = nRF_TX_PWR_0dBm;
  myNRF.State = nRF_STATE_TX;

  myNRF.RF_Channel = 2;
  myNRF.PayloadWidth = nRF_RXPW_32BYTES;
  myNRF.RetransmitCount = nRF_RETX_COUNT_1;
  myNRF.RetransmitDelay = nRF_RETX_DELAY_1000uS;

  myNRF.RX_Address = (uint8_t *)RXAddr;
  myNRF.TX_Address = (uint8_t *)TXAddr;

  myNRF.RX_Buffer = RXBuffer;
  myNRF.TX_Buffer = TXBuffer;

  myNRF.nRF_nSS_GPIO_PORT = nRF_nSS_GPIO_Port;
  myNRF.nRF_nSS_GPIO_PIN = nRF_nSS_Pin;

  myNRF.nRF_CE_GPIO_PORT = nRF_CE_GPIO_Port;
  myNRF.nRF_CE_GPIO_PIN = nRF_CE_Pin;
  
  if(HAL_nRF24L01P_Init(&myNRF) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    read_MPU6050_data();
    
    if(Accel_X <= -accel_bound)
    {
      TXData[0] = 'F';
    }else if(Accel_X >= accel_bound)
    {
      TXData[0] = 'B';
    }else if(Accel_Y <= -accel_bound)
    {
      TXData[0] = 'R';
    }else if(Accel_Y >= accel_bound)
    {
      TXData[0] = 'L';
    }else
    {
      TXData[0] = 'D';
    }
    if(HAL_nRF24L01P_TransmitPacket(&myNRF, (uint8_t *)TXData) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    HAL_Delay(250);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nRF_CE_Pin|nRF_nSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : nRF_IRQ_Pin */
  GPIO_InitStruct.Pin = nRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF_CE_Pin nRF_nSS_Pin */
  GPIO_InitStruct.Pin = nRF_CE_Pin|nRF_nSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_2)
  {
    if(HAL_nRF24L01P_IRQ_Handler(&myNRF) != HAL_OK)
    {
      puts("ERROR : HAL_GPIO_EXTI_Callback");
      while(1){}
    }
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
