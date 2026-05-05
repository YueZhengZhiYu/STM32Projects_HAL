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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "../.././../EXTI/Drivers/OLED/OLED.h"
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
volatile uint8_t key_flag[4] = {0}; 

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_ALL_TOGGLE,   // PB3
    KEY_EVENT_1_4_TOGGLE,   // PB5
    KEY_EVENT_2_3_TOGGLE    // PB12&PB14
} KeyEvent_t;

volatile KeyEvent_t key_press_event = KEY_EVENT_NONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case KEY_3_Pin:  // PB3
            key_press_event = KEY_EVENT_ALL_TOGGLE;
            break;

        case KEY_4_Pin:  // PB5
            key_press_event = KEY_EVENT_1_4_TOGGLE;
            break;

        case KEY_1_Pin:  // PB12
        case KEY_2_Pin:  // PB14
            key_press_event = KEY_EVENT_2_3_TOGGLE;
            break;

        default:
            break;
    }
}

/*
 * 按键处理与LED控制函数：在主循环中调用。
 * 包含安全延时，不会卡死。
 */
void ProcessKeyAndControlLED(void)
{
    switch(key_press_event)
    {
        case KEY_EVENT_ALL_TOGGLE:
            // 软件去抖延时，在主循环中执行是安全的
            HAL_Delay(100);
            // 再次确认引脚电平，确保是有效按下
            if(HAL_GPIO_ReadPin(GPIOB, KEY_3_Pin) == GPIO_PIN_RESET)
            {
                HAL_GPIO_TogglePin(GPIOA, LED_1_Pin | LED_2_Pin | LED_3_Pin | LED_4_Pin);
            }
            break;

        case KEY_EVENT_1_4_TOGGLE:
            HAL_Delay(100);
            if(HAL_GPIO_ReadPin(GPIOB, KEY_4_Pin) == GPIO_PIN_RESET)
            {
                HAL_GPIO_TogglePin(GPIOA, LED_1_Pin | LED_4_Pin);
            }
            break;

        case KEY_EVENT_2_3_TOGGLE:
            HAL_Delay(100);
            // PB12和PB14功能相同，检查其中任何一个即可，或都检查
            if(HAL_GPIO_ReadPin(GPIOB, KEY_1_Pin) == GPIO_PIN_RESET ||
               HAL_GPIO_ReadPin(GPIOB, KEY_2_Pin) == GPIO_PIN_RESET)
            {
                HAL_GPIO_TogglePin(GPIOA, LED_2_Pin | LED_3_Pin);
            }
            break;

        case KEY_EVENT_NONE:
        default:
            // 没有事件，不做任何事
            break;
    }
    
    // 处理完事件后，清除标志，等待下一次触发
    key_press_event = KEY_EVENT_NONE;
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
  /* USER CODE BEGIN 2 */
OLED_Init();
	  OLED_Clear();
  
      OLED_Clear();
	  
	  OLED_ShowCHinese(0,0,20,0);
	  OLED_ShowCHinese(16,0,21,0);
	  OLED_ShowCHinese(32,0,22,0);
	  OLED_ShowCHinese(48,0,9,0);
	  OLED_ShowCHinese(64,0,10,0);
	  
	  /*第二行*/
	  OLED_ShowCHinese(0,3,11,0); 
	  OLED_ShowCHinese(16,3,12,0); 
	  OLED_ShowString(32,3,"A236112",12,0);
	  OLED_ShowCHinese(80,3,17,0); 
	  OLED_ShowCHinese(96,3,18,0); 
	  OLED_ShowCHinese(112,3,19,0); 
	  
	  /*第三行*/
	  OLED_ShowString(0,6,"2026.4.27",16,0);

	  HAL_Delay(10000);
	  
	  OLED_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
