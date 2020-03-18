/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2s.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cctalk.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */
   
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 #define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000



struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
      if (DEMCR & TRCENA)
				{
        while (ITM_Port32(0) == 0);
        ITM_Port8(0) = ch;
        }
  return(ch);
        
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    //  uint8_t str[]="USART Transmit DMA\r\n";
    extern DMA_HandleTypeDef hdma_usart1_tx;
    extern DMA_HandleTypeDef hdma_usart1_rx;
    extern DMA_HandleTypeDef hdma_spi1_tx;
    extern DMA_HandleTypeDef hdma_spi2_rx;
    extern DMA_HandleTypeDef hdma_spi3_tx;
    uint8_t income[512];
  //  uint8_t income[256];
    //uint16_t syne_wave[512];
 //   uint16_t  *ptr = syne_wave;
    uint16_t syne,index;
    uint32_t crc;
    uint8_t  crc_cctalk;
    uint8_t devid_cmd[1] = { 0x9F };
    uint8_t flash_read_cmd[4];
    flash_read_cmd[0]=0x0b;
    flash_read_cmd[1]=0;
    flash_read_cmd[2]=0;
    flash_read_cmd[3]=0;
    const float  pi = 3.1415927;
 
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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2S3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  _LED2_OFF_;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)cmd01, sizeof(cmd01)); 
    while (hdma_usart1_tx.State==HAL_DMA_STATE_BUSY);
    HAL_Delay(500);  
   //check SPI FLASH ID
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, devid_cmd, 1,500);
   HAL_SPI_Receive(&hspi2,income,5,500);
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
   HAL_Delay(10);
   /*
   //-------------------------------------------------------------
       for (int i=0;i<512/2;i++) {
       
           syne = sin(  (2*pi / 256 ) *i ) * 30000;
           *ptr++= syne;
           *ptr++ = syne;           
       
       
       }
   
   
   //-------------------------------------------------------------
     */
  index=0;
//   read first 256 bytes from flash
   for (int j=0; j<0x3a; j++) {

   for (int i=0; i<256;i+=2) {
   flash_read_cmd[2]=i;    
    flash_read_cmd[1]=j;    
   
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
   HAL_SPI_Transmit_DMA(&hspi2, flash_read_cmd,sizeof(flash_read_cmd)+1);
   while(hdma_spi2_rx.State != HAL_DMA_STATE_READY);

   HAL_SPI_Receive_DMA(&hspi2,income,512);
   while(hdma_spi2_rx.State != HAL_DMA_STATE_READY);
 
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
  
   
   HAL_I2S_Transmit(&hi2s3,(uint16_t *) &income[0],sizeof(income)/2,10000);
   while (hdma_spi3_tx.State !=HAL_DMA_STATE_READY);
       
   //HAL_Delay(1);    
   }
    }
   HAL_Delay(10);
   
   
   
   
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_UART_Receive_DMA(&huart1, income,21);  
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)cmdE5, sizeof(cmdE5));  
  _LED1_ON_;    
  while (hdma_usart1_tx.State==HAL_DMA_STATE_BUSY);
  //printf("Sent \n\r");    
   //   HAL_Delay(1);
      
  
  while (hdma_usart1_rx.State!=HAL_DMA_STATE_READY);     
  crc=0;
  for (int i=5;i<20;i++)
      {crc+=income[i];
      }
  crc_cctalk=(uint8_t)(256-(crc%256));    
  printf("crc %x\n\r",crc_cctalk);
      
  _LED1_OFF_;      
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
