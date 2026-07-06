#include "DHT11.h"

#define DHT_PORT    DHT_DATA_GPIO_Port
#define DHT_PIN     DHT_DATA_Pin
#define DHT_IN()    do { DHT_PORT->CRL &= ~0xF00000; DHT_PORT->CRL |= 0x800000; \
                         DHT_PORT->BSRR = DHT_PIN; } while(0)
#define DHT_OUT()   do { DHT_PORT->CRL &= ~0xF00000; DHT_PORT->CRL |= 0x700000; } while(0)
#define DHT_LOW()   (DHT_PORT->BRR = DHT_PIN)
#define DHT_HIGH()  (DHT_PORT->BSRR = DHT_PIN)
#define DHT_READ()  ((DHT_PORT->IDR & DHT_PIN) ? 1 : 0)

static void delay_us(uint16_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 4000000);
    while (ticks--) {
        __NOP();
    }
}

static uint8_t wait_level(uint8_t level, uint16_t timeout_us)
{
    while (timeout_us--) {
        if (DHT_READ() == level) return 1;
        delay_us(1);
    }
    return 0;
}

static uint8_t read_byte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;

        if (!wait_level(1, 200)) return byte;

        uint16_t high_cnt = 0;
        while (DHT_READ()) {
            high_cnt++;
            if (high_cnt > 2000) break;
        }

        if (high_cnt > 300) byte |= 1;
    }
    return byte;
}

uint8_t DHT11_Read(DHT11_Data *data)
{
    uint8_t buf[5] = {0};

    for (uint8_t retry = 0; retry < 3; retry++) {
        __disable_irq();

        DHT_OUT();
        DHT_LOW();
        delay_us(18000);
        DHT_HIGH();
        delay_us(30);

        DHT_IN();

        if (!wait_level(0, 200)) { __enable_irq(); continue; }
        if (!wait_level(1, 200)) { __enable_irq(); continue; }

        if (!wait_level(0, 200)) { __enable_irq(); continue; }

        for (uint8_t i = 0; i < 5; i++) {
            buf[i] = read_byte();
        }

        __enable_irq();

        if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            if (buf[0] == 0 && buf[2] == 0) continue;
            data->humidity = (float)buf[0];
            data->temperature = (float)buf[2];
            return DHT11_OK;
        }
    }
    return DHT11_CHKFAIL;
}
