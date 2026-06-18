#ifndef __DHT11_H__
#define __DHT11_H__

#include "main.h"

#define DHT11_OK       0
#define DHT11_TIMEOUT  1
#define DHT11_CHKFAIL  2

typedef struct {
    float temperature;
    float humidity;
} DHT11_Data;

uint8_t DHT11_Read(DHT11_Data *data);

#endif
