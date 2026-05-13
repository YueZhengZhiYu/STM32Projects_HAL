/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "../../../UART/Drivers/OLED/OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


// 串口接收缓冲区
uint8_t USART_RX_BUF[10];  // 接收缓冲区，最大10字节
uint8_t USART_RX_CNT = 0;  // 接收字节数
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
void Cmd_Parse(void)
{
    // 比较接收的指令（忽略大小写，此处仅匹配大写）
    if(USART_RX_CNT == 6 && memcmp(USART_RX_BUF, "LED_ON", 6) == 0)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // 点亮LED
        USART_RX_CNT = 0;  // 清空计数
		
		OLED_Clear();
        OLED_ShowString(0, 2, "LED: ON", 16, 0);
		
		Serial_Printf("LED 已点亮\n\r");

        USART_RX_CNT = 0;
        memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));  // 清空缓冲区
	}
    else if(USART_RX_CNT == 7 && memcmp(USART_RX_BUF, "LED_OFF", 7) == 0)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);   // 熄灭LED
        USART_RX_CNT = 0;
		
		OLED_Clear();
        OLED_ShowString(0, 2, "LED: OFF", 16, 0);
		
		Serial_Printf("LED 已熄灭\n\r");

        USART_RX_CNT = 0;
        memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
    }
    // 超出缓冲区或指令错误，清空
    else if(USART_RX_CNT >= 10)
    {
        USART_RX_CNT = 0;
        memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	  OLED_Init();
	  OLED_Clear();
  
      OLED_Clear();
	  
	  OLED_ShowCHinese(0,0,17,0);
	  OLED_ShowCHinese(16,0,18,0);
	  OLED_ShowCHinese(32,0,19,0);
	  OLED_ShowCHinese(48,0,20,0);
	  OLED_ShowCHinese(64,0,9,0);
	  OLED_ShowCHinese(80,0,10,0);
	  
	  /*第二行*/
	  OLED_ShowCHinese(0,3,11,0); 
	  OLED_ShowCHinese(16,3,12,0); 
	  OLED_ShowString(32,3,"A236144",16,0);
	  OLED_ShowCHinese(80,3,24,0);  
	  OLED_ShowCHinese(96,3,25,0); 
	  OLED_ShowCHinese(112,3,26,0);
	  
	  /*第三行*/
	  OLED_ShowString(0,6,"2026.5.18",16,0);

	  HAL_Delay(10000);
	  
	  OLED_Clear();
	
  HAL_UART_Receive_IT(&huart1, &USART_RX_BUF[USART_RX_CNT], 1);

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	Cmd_Parse();  // 指令解析
    HAL_Delay(10);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(USART_RX_CNT < 10)
        {
			USART_RX_BUF[USART_RX_CNT] = huart->Instance->DR;
            USART_RX_CNT++;  // 收到1个字节，计数+1
        }
        
        // 继续等待下一个字节
        HAL_UART_Receive_IT(&huart1, &USART_RX_BUF[USART_RX_CNT], 1);
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
#ifdef USE_FULL_ASSERT
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
