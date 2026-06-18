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
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "DHT11.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prev_error;
    float out_min, out_max;
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_ALARM_HIGH         26.0f
#define TEMP_ALARM_LOW          24.0f
#define HUM_ALARM_HIGH          65.0f
#define HUM_ALARM_LOW           60.0f
#define SERVO_ANGLE_MIN         0
#define SERVO_ANGLE_MAX         180
#define PWM_SERVO_MIN           50
#define PWM_SERVO_MAX           250
#define SEND_INTERVAL_S         60
#define SENSOR_INTERVAL_MS      2000
#define DEBOUNCE_MS             20
#define STARTUP_SCREEN1_MS      3000
#define STARTUP_SCREEN2_MS      5000

#define PID_KP_TEMP             40.0f
#define PID_KI_TEMP             2.0f
#define PID_KD_TEMP             5.0f
#define PID_SETPOINT_TEMP       ((TEMP_ALARM_HIGH + TEMP_ALARM_LOW) / 2.0f)
#define PID_KP_HUM              30.0f
#define PID_KI_HUM              1.0f
#define PID_KD_HUM              3.0f
#define PID_SETPOINT_HUM        65.0f
#define PID_OUT_MIN             0.0f
#define PID_OUT_MAX             19999.0f

#define BUZZ_ON()               HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET)
#define BUZZ_OFF()              HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t  auto_mode = 0;
static float    temp_sum = 0.0f;
static float    hum_sum = 0.0f;
static uint16_t sample_count = 0;
static float    avg_temp = 0.0f;
static float    avg_hum = 0.0f;
static float    current_temp = 0.0f;
static float    current_hum = 0.0f;
static uint16_t fan_duty = 0;
static uint16_t servo_angle = 90;
static uint8_t  alarm_state = 0;
static char     usart_tx_buf[64];
static uint32_t last_sensor_tick = 0;
static uint32_t last_send_tick = 0;
static uint8_t  main_press = 1;
static uint8_t  mode_sel = 1;
static PID_t     pid_temp;
static PID_t     pid_hum;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t  Button_ScanAdd(void);
static uint8_t  Button_ScanSub(void);
static void     PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float setpoint, float out_min, float out_max);
static float    PID_Update(PID_t *pid, float input, float dt);
static uint16_t PID_FanControl(float temp, float hum);
static void     UpdateOLED_Screen1(void);
static void     UpdateOLED_Screen2(uint8_t mode_sel, uint32_t remaining_s);
static void     UpdateOLED_Main(float temp, float hum, uint16_t fan_duty_val, uint16_t servo_val);
static void     ParseUSARTCommand(char *cmd);
static void     USART_SendSensorData(void);
static void     CheckAlarm(float temp, float hum);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t Button_ScanAdd(void)
{
    static uint8_t  last_stable = 1;
    static uint8_t  last_raw = 1;
    static uint32_t tick = 0;
    uint8_t state = HAL_GPIO_ReadPin(KEY_ADD_GPIO_Port, KEY_ADD_Pin);
    if (state != last_raw) {
        tick = HAL_GetTick();
        last_raw = state;
    }
    if ((HAL_GetTick() - tick) >= DEBOUNCE_MS) {
        if (last_raw == 0 && last_stable == 1) {
            last_stable = 0;
            return 1;
        }
        if (last_raw == 1) {
            last_stable = 1;
        }
    }
    return 0;
}

static uint8_t Button_ScanSub(void)
{
    static uint8_t  last_stable = 1;
    static uint8_t  last_raw = 1;
    static uint32_t tick = 0;
    uint8_t state = HAL_GPIO_ReadPin(KEY_SUB_GPIO_Port, KEY_SUB_Pin);
    if (state != last_raw) {
        tick = HAL_GetTick();
        last_raw = state;
    }
    if ((HAL_GetTick() - tick) >= DEBOUNCE_MS) {
        if (last_raw == 0 && last_stable == 1) {
            last_stable = 0;
            return 1;
        }
        if (last_raw == 1) {
            last_stable = 1;
        }
    }
    return 0;
}

static void LED_UpdateByAngle(uint16_t angle)
{
    uint8_t idx;
    if      (angle < 30)  idx = 0;
    else if (angle < 90)  idx = 1;
    else if (angle < 150) idx = 2;
    else                  idx = 3;

    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (idx == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (idx == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (idx == 2) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, (idx == 3) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float setpoint, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

static float PID_Update(PID_t *pid, float input, float dt)
{
    float error = pid->setpoint - input;
    pid->integral += error * dt;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < 0.0f) pid->integral = 0.0f;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    return output;
}

static uint16_t PID_FanControl(float temp, float hum)
{
    float dt = SENSOR_INTERVAL_MS / 1000.0f;
    float out_temp = PID_Update(&pid_temp, temp, dt);
    float out_hum  = PID_Update(&pid_hum,  hum,  dt);

    if (temp <= PID_SETPOINT_TEMP && out_temp < 100.0f) out_temp = 0.0f;
    if (hum <= PID_SETPOINT_HUM && out_hum < 100.0f)    out_hum  = 0.0f;

    if (temp > PID_SETPOINT_TEMP && out_temp < 2000.0f) out_temp = 2000.0f;
    if (hum > PID_SETPOINT_HUM && out_hum < 2000.0f)    out_hum  = 2000.0f;

    float output = (out_temp > out_hum) ? out_temp : out_hum;
    if (output > PID_OUT_MAX) output = PID_OUT_MAX;
    if (output < 0.0f) output = 0.0f;
    return (uint16_t)output;
}

static void UpdateOLED_Screen1(void)
{
    OLED_Clear();
    OLED_ShowCHinese(24, 0, 4, 0);
    OLED_ShowCHinese(40, 0, 5, 0);
    OLED_ShowCHinese(56, 0, 32, 0);
    OLED_ShowCHinese(72, 0, 33, 0);
    OLED_ShowString(24, 4, "A236112 ZhangXM", 16, 0);
}

static void UpdateOLED_Screen2(uint8_t mode_sel, uint32_t remaining_s)
{
    static uint8_t  last_sel = 0xFF;
    static uint32_t last_rem = 0xFFFFFFFF;

    if (mode_sel == last_sel && remaining_s == last_rem) return;
    last_sel = mode_sel; last_rem = remaining_s;

    OLED_Clear();
    OLED_ShowString(0, 0, "Select Mode", 16, 0);
    if (mode_sel) {
        OLED_ShowString(0, 2, ">Manual Mode", 16, 0);
        OLED_ShowString(0, 4, " Auto Mode", 16, 0);
    } else {
        OLED_ShowString(0, 2, ">Auto Mode", 16, 0);
        OLED_ShowString(0, 4, " Manual Mode", 16, 0);
    }
    char buf[14];
    sprintf(buf, "Countdown:%lu", remaining_s);
    OLED_ShowString(0, 6, buf, 16, 0);
}

static void UpdateOLED_Main(float temp, float hum, uint16_t fan_duty_val, uint16_t servo_val)
{
    static float    last_temp = -99.0f, last_hum = -99.0f;
    static uint16_t last_duty = 0xFFFF, last_servo = 0xFFFF;

    if (temp == last_temp && hum == last_hum && fan_duty_val == last_duty && servo_val == last_servo)
    {
        return;
    }
    last_temp = temp; last_hum = hum; last_duty = fan_duty_val; last_servo = servo_val;

    OLED_Clear();

    OLED_ShowCHinese(0, 0, 23, 0);
    OLED_ShowCHinese(16, 0, 25, 0);
    char buf[12];
    sprintf(buf, ": %.1f C", temp);
    OLED_ShowString(32, 0, buf, 16, 0);

    OLED_ShowCHinese(0, 2, 24, 0);
    OLED_ShowCHinese(16, 2, 25, 0);
    sprintf(buf, ": %.1f %%", hum);
    OLED_ShowString(32, 2, buf, 16, 0);

    OLED_ShowCHinese(0, 4, 26, 0);
    OLED_ShowCHinese(16, 4, 27, 0);
    OLED_ShowString(34, 4, ":", 16, 0);
    OLED_ShowNum(42, 4, fan_duty_val, 5, 16, 0);

    OLED_ShowString(0, 6, "Servo:", 16, 0);
    OLED_ShowNum(48, 6, servo_val, 3, 16, 0);
}

static void ParseUSARTCommand(char *cmd)
{
    if (strncmp(cmd, "M:0", 3) == 0) { auto_mode = 1; mode_sel = 0; }
    else if (strncmp(cmd, "M:1", 3) == 0) { auto_mode = 0; mode_sel = 1; }
    else if (strncmp(cmd, "S:", 2) == 0) {
        int ang = atoi(cmd + 2);
        if (ang < SERVO_ANGLE_MIN) ang = SERVO_ANGLE_MIN;
        if (ang > SERVO_ANGLE_MAX) ang = SERVO_ANGLE_MAX;
        servo_angle = (uint16_t)ang;
        uint16_t cmp = PWM_SERVO_MIN + (uint32_t)(servo_angle) * (PWM_SERVO_MAX - PWM_SERVO_MIN) / 180;
        PWM_SetServoCompare(cmp);
        LED_UpdateByAngle(servo_angle);
    }
    else if (strncmp(cmd, "F:", 2) == 0) {
        int pct = atoi(cmd + 2);
        if (pct < 0) pct = 0; if (pct > 100) pct = 100;
        fan_duty = (uint16_t)(pct * 19999 / 100);
        if (fan_duty > 0) {
            HAL_GPIO_WritePin(TB6612_INA_1_GPIO_Port, TB6612_INA_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(TB6612_INA_2_GPIO_Port, TB6612_INA_2_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(TB6612_INA_1_GPIO_Port, TB6612_INA_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(TB6612_INA_2_GPIO_Port, TB6612_INA_2_Pin, GPIO_PIN_RESET);
        }
        PWM_SetDriverCompare(fan_duty);
    }
}

static void USART_SendSensorData(void)
{
    sprintf(usart_tx_buf, "T:%.1f,H:%.1f,AVGT:%.1f,AVGH:%.1f\r\n",
            current_temp, current_hum, avg_temp, avg_hum);
    USART_SendString(usart_tx_buf);
}

static void CheckAlarm(float temp, float hum)
{
    uint8_t should_alarm = 0;
    if (temp >= TEMP_ALARM_HIGH || hum >= HUM_ALARM_HIGH) {
        should_alarm = 1;
    } else if (temp <= TEMP_ALARM_LOW && hum <= HUM_ALARM_LOW) {
        should_alarm = 0;
    } else if (alarm_state) {
        should_alarm = 1;
    }

    if (should_alarm && !alarm_state) {
        BUZZ_ON();
        alarm_state = 1;
    } else if (!should_alarm && alarm_state) {
        BUZZ_OFF();
        alarm_state = 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  TIM1->BDTR |= TIM_BDTR_MOE;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  PID_Init(&pid_temp, PID_KP_TEMP, PID_KI_TEMP, PID_KD_TEMP, PID_SETPOINT_TEMP, PID_OUT_MIN, PID_OUT_MAX);
  PID_Init(&pid_hum,  PID_KP_HUM,  PID_KI_HUM,  PID_KD_HUM,  PID_SETPOINT_HUM,  PID_OUT_MIN, PID_OUT_MAX);

  PWM_SetServoCompare(PWM_SERVO_MIN + (uint32_t)(servo_angle) * (PWM_SERVO_MAX - PWM_SERVO_MIN) / 180);
  PWM_SetDriverCompare(0);

  HAL_GPIO_WritePin(TB6612_INA_1_GPIO_Port, TB6612_INA_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TB6612_INA_2_GPIO_Port, TB6612_INA_2_Pin, GPIO_PIN_RESET);
  BUZZ_OFF();
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

  UpdateOLED_Screen1();
  HAL_Delay(STARTUP_SCREEN1_MS);

  mode_sel = 1;
  uint32_t screen2_start = HAL_GetTick();
  uint32_t screen2_elapsed;
  uint32_t screen2_remaining;

  servo_angle = 0;
  PWM_SetServoCompare(PWM_SERVO_MIN);
  LED_UpdateByAngle(servo_angle);

  do {
      screen2_elapsed = HAL_GetTick() - screen2_start;
      screen2_remaining = (STARTUP_SCREEN2_MS - screen2_elapsed + 999) / 1000;
      uint8_t add = Button_ScanAdd();
      uint8_t sub = Button_ScanSub();
      if (add) {
          mode_sel = !mode_sel;
      }
      if (sub) {
          if (servo_angle == 0) servo_angle = 180;
          else                  servo_angle -= 60;
      }
      if (add || sub) {
          uint16_t cmp = PWM_SERVO_MIN + (uint32_t)(servo_angle) * (PWM_SERVO_MAX - PWM_SERVO_MIN) / 180;
          PWM_SetServoCompare(cmp);
          LED_UpdateByAngle(servo_angle);
      }
      UpdateOLED_Screen2(mode_sel, screen2_remaining);
  } while (screen2_elapsed < STARTUP_SCREEN2_MS);

  servo_angle = 0;
  main_press = 1;
  PWM_SetServoCompare(PWM_SERVO_MIN);
  LED_UpdateByAngle(servo_angle);

  if (mode_sel == 0) {
      auto_mode = 1;
  } else {
      auto_mode = 0;
  }

  last_send_tick = HAL_GetTick();
  last_sensor_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t tick = HAL_GetTick();

    if (tick - last_sensor_tick >= SENSOR_INTERVAL_MS)
    {
        last_sensor_tick = tick;

        DHT11_Data dht_data;
        if (DHT11_Read(&dht_data) == DHT11_OK)
        {
            float new_temp = dht_data.temperature;
            float new_hum  = dht_data.humidity;
            if (current_temp > 0.0f) {
                float dt = new_temp - current_temp;
                float dh = new_hum - current_hum;
                if (dt > 10.0f || dt < -10.0f || dh > 30.0f || dh < -30.0f) {
                    new_temp = current_temp;
                    new_hum  = current_hum;
                }
            }
            current_temp = new_temp;
            current_hum  = new_hum;
            if (current_hum > 100.0f) current_hum = 100.0f;
            if (current_temp > TEMP_ALARM_HIGH) current_temp = TEMP_ALARM_HIGH;
            if (current_temp < TEMP_ALARM_LOW)  current_temp = TEMP_ALARM_LOW;
        }
        temp_sum += current_temp;
        hum_sum  += current_hum;
        sample_count++;

        if (auto_mode)
        {
            fan_duty = PID_FanControl(current_temp, current_hum);
            if (fan_duty > 0)
            {
                HAL_GPIO_WritePin(TB6612_INA_1_GPIO_Port, TB6612_INA_1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(TB6612_INA_2_GPIO_Port, TB6612_INA_2_Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(TB6612_INA_1_GPIO_Port, TB6612_INA_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TB6612_INA_2_GPIO_Port, TB6612_INA_2_Pin, GPIO_PIN_RESET);
            }
            PWM_SetDriverCompare(fan_duty);
        }

        CheckAlarm(current_temp, current_hum);

        UpdateOLED_Main(current_temp, current_hum, fan_duty, servo_angle);
    }

    if (!auto_mode)
    {
        uint8_t add = Button_ScanAdd();
        uint8_t sub = Button_ScanSub();
        if (add) {
            servo_angle += 60;
            if (servo_angle > 180) servo_angle = 0;
        }
        if (sub) {
            if (servo_angle == 0) servo_angle = 180;
            else                  servo_angle -= 60;
        }
        if (add || sub) {
            main_press = servo_angle / 60 + 1;
            uint16_t cmp = PWM_SERVO_MIN + (uint32_t)(servo_angle) * (PWM_SERVO_MAX - PWM_SERVO_MIN) / 180;
            PWM_SetServoCompare(cmp);
            LED_UpdateByAngle(servo_angle);
            UpdateOLED_Main(current_temp, current_hum, fan_duty, servo_angle);
        }
    }

    if (tick - last_send_tick >= SEND_INTERVAL_S * 1000)
    {
        last_send_tick = tick;
        if (sample_count > 0)
        {
            avg_temp = temp_sum / sample_count;
            avg_hum  = hum_sum  / sample_count;
        }
        temp_sum = 0.0f;
        hum_sum  = 0.0f;
        sample_count = 0;
        USART_SendSensorData();
    }

    if (usart_rx_flag)
    {
        ParseUSARTCommand((char *)usart_rx_buf);
        usart_rx_flag = 0;
    }

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
