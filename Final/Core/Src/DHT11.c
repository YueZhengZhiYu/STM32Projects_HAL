#include "DHT11.h"

static uint8_t dht_dwt_ready = 0;

static void DHT11_Delay_us(uint16_t us)
{
    if (!dht_dwt_ready) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dht_dwt_ready = 1;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

static void DHT11_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT_DATA_GPIO_Port, &GPIO_InitStruct);
}

static void DHT11_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT_DATA_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t DHT11_WaitLevel(uint8_t level, uint16_t timeout_us)
{
    while (timeout_us--) {
        if (HAL_GPIO_ReadPin(DHT_DATA_GPIO_Port, DHT_DATA_Pin) == level) {
            return 1;
        }
        DHT11_Delay_us(1);
    }
    return 0;
}

static uint8_t DHT11_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        byte <<= 1;

        if (!DHT11_WaitLevel(GPIO_PIN_SET, 100))
        {
            return byte;
        }

        DHT11_Delay_us(40);

        if (HAL_GPIO_ReadPin(DHT_DATA_GPIO_Port, DHT_DATA_Pin) == GPIO_PIN_SET)
        {
            byte |= 1;
        }

        if (!DHT11_WaitLevel(GPIO_PIN_RESET, 120))
        {
            break;
        }
    }
    return byte;
}

uint8_t DHT11_Read(DHT11_Data *data)
{
    uint8_t buf[5] = {0};
    uint16_t retry = 3;

    while (retry--) {
        __disable_irq();

        DHT11_SetOutput();
        HAL_GPIO_WritePin(DHT_DATA_GPIO_Port, DHT_DATA_Pin, GPIO_PIN_RESET);
        DHT11_Delay_us(18000);
        HAL_GPIO_WritePin(DHT_DATA_GPIO_Port, DHT_DATA_Pin, GPIO_PIN_SET);
        DHT11_Delay_us(30);

        DHT11_SetInput();

        if (!DHT11_WaitLevel(GPIO_PIN_RESET, 150)) {
            __enable_irq();
            HAL_Delay(100);
            continue;
        }
        if (!DHT11_WaitLevel(GPIO_PIN_SET, 150)) {
            __enable_irq();
            HAL_Delay(100);
            continue;
        }

        for (uint8_t i = 0; i < 5; i++) {
            buf[i] = DHT11_ReadByte();
        }

        __enable_irq();

        if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            data->humidity = (float)buf[0];
            if (data->humidity > 100.0f) data->humidity = 100.0f;

            data->temperature = (float)buf[2];

            return DHT11_OK;
        }

        HAL_Delay(100);
    }
    return DHT11_CHKFAIL;
}
