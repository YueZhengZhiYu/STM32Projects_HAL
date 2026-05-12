/* USER CODE BEGIN Header */
/* USER CODE END Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include <OLED.h>
#include <stdio.h>

uint8_t KeyNum;
int16_t Count = 2;          // 初始计数 2 → 角度 90°
uint8_t key_flag = 0;

// 角度映射表：余数 0~4 -> 对应角度 0,45,90,135,180
const uint8_t angleMap[5] = {0, 45, 90, 135, 180};

// 脉冲宽度映射表（单位：定时器计数周期，1us 一个计数）
// 0°  = 500us  → 500
// 45° = 1000us → 1000
// 90° = 1500us → 1500
// 135°= 2000us → 2000
// 180°= 2500us → 2500
const uint16_t pulseMap[5] = {500, 1000, 1500, 2000, 2500};

// 根据 Count 获取角度和脉冲
uint8_t GetAngleFromCount(int16_t count)
{
    int16_t mod = count % 5;
    if (mod < 0) mod += 5;
    return angleMap[mod];
}

uint16_t GetPulseFromCount(int16_t count)
{
    int16_t mod = count % 5;
    if (mod < 0) mod += 5;
    return pulseMap[mod];
}

// 按键扫描
uint8_t Key_GetNum(void)
{
    uint8_t key = 0;
    if (key_flag == 0)
    {
        if (HAL_GPIO_ReadPin(GPIOB, KEY_Add_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(GPIOB, KEY_Add_Pin) == GPIO_PIN_RESET)
            {
                key = 1;
                key_flag = 1;
            }
        }
        else if (HAL_GPIO_ReadPin(GPIOB, KEY_Subtract_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(GPIOB, KEY_Subtract_Pin) == GPIO_PIN_RESET)
            {
                key = 2;
                key_flag = 1;
            }
        }
    }
    if (HAL_GPIO_ReadPin(GPIOB, KEY_Add_Pin) != GPIO_PIN_RESET &&
        HAL_GPIO_ReadPin(GPIOB, KEY_Subtract_Pin) != GPIO_PIN_RESET)
    {
        key_flag = 0;
    }
    return key;
}

// 舵机驱动：直接设置比较值
void Servo_SetPulse(uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

// LED 控制（按 Count % 4 亮灯）
void LED_Control(int16_t cnt)
{
    int16_t mode = cnt % 4;
    if (mode < 0) mode += 4;

    HAL_GPIO_WritePin(GPIOA, LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin, GPIO_PIN_SET);
    switch(mode)
    {
        case 1: HAL_GPIO_WritePin(GPIOA, LED_1_Pin, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(GPIOA, LED_3_Pin, GPIO_PIN_RESET); break;
        case 0: HAL_GPIO_WritePin(GPIOA, LED_4_Pin, GPIO_PIN_RESET); break;
    }
}

void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();                     // tim.c 已修正 Period 和极性

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 0, "Angle:", 16, 0);
    OLED_ShowString(1, 3, "Cnt:", 16, 0);
    OLED_ShowString(1, 6, "Pulse:", 16, 0);

    // 初始显示
    uint8_t angle = GetAngleFromCount(Count);
    uint16_t pulse = GetPulseFromCount(Count);
    Servo_SetPulse(pulse);
    LED_Control(Count);

    char buf[20];
    sprintf(buf, "%3d", angle);
    OLED_ShowString(48, 1, buf, 15, 0);
    sprintf(buf, "%4d", Count);
    OLED_ShowString(32, 4, buf, 15, 0);
    sprintf(buf, "%4d", pulse);
    OLED_ShowString(56, 7, buf, 15, 0);

    while (1)
    {
        KeyNum = Key_GetNum();

        if (KeyNum == 1)      // 右键：计数 +1
        {
            Count++;
        }
        if (KeyNum == 2)      // 左键：计数 -1
        {
            Count--;
        }

        // 更新舵机和 LED
        uint16_t newPulse = GetPulseFromCount(Count);
        Servo_SetPulse(newPulse);
        LED_Control(Count);

        // 刷新 OLED
        static int16_t oldCount = 0;
        static uint16_t oldPulse = 0;
        if (oldCount != Count || oldPulse != newPulse)
        {
            oldCount = Count;
            oldPulse = newPulse;

            uint8_t angle = GetAngleFromCount(Count);
            char buf[20];

            sprintf(buf, "%3d", angle);
            OLED_ShowString(48, 1, buf, 15, 0);
            sprintf(buf, "%4d", Count);
            OLED_ShowString(32, 4, buf, 15, 0);
            sprintf(buf, "%4d",  newPulse);
            OLED_ShowString(56, 7, buf, 15, 0);
        }
        HAL_Delay(20);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void) {}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
	
