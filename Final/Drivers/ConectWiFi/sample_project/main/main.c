#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "mqtt_client.h"

#define UART_PORT           UART_NUM_1
#define UART_TX_PIN         39
#define UART_RX_PIN         38
#define UART_BAUD           115200
#define UART_BUF_SIZE       256

#define WIFI_SSID           "Your_WiFi_ID"
#define WIFI_PASSWORD       "Password"
#define WIFI_MAX_RETRY      5

/* ---------- OneNET MQTT 配置 ---------- */
#define ONENET_PRODUCT_ID   "Your_OneNET_Product_ID"
#define ONENET_DEVICE_NAME  "Your_OneNET_Product_Name"
#define ONENET_ACCESS_KEY   "Your_OneNET_Product_KEY"

#define TAG                 "STM32_COMM"

static int  g_wifi_connected = 0;
static int  g_wifi_retry = 0;
static EventGroupHandle_t g_evt = NULL;
#define WIFI_CONNECTED_BIT  BIT0
#define TIME_SYNCED_BIT     BIT1

static void time_sync_cb(struct timeval *tv)
{
    xEventGroupSetBits(g_evt, TIME_SYNCED_BIT);
}

static esp_mqtt_client_handle_t g_mqtt_client = NULL;

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "OneNET MQTT connected!");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT cmd: %.*s", ev->data_len, (char *)ev->data);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

static void mqtt_init(void)
{
    xEventGroupWaitBits(g_evt, TIME_SYNCED_BIT, pdFALSE, pdTRUE,
                        pdMS_TO_TICKS(30000));

    char pub_topic[128];
    snprintf(pub_topic, sizeof(pub_topic),
             "$sys/%s/%s/thing/property/post",
             ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtt://mqtts.heclouds.com:1883",
        .credentials = {
            .client_id = ONENET_DEVICE_NAME,
            .username = ONENET_PRODUCT_ID,
            .authentication.password = "version=2018-10-31&res=products%2FINidf1Z75p%2Fdevices%2FESP32_Test&et=1908112656&method=md5&sign=gKfYDcrkKR3eMCqDjIp2CQ%3D%3D",
        },
        .session.keepalive = 120,
        .network.disable_auto_reconnect = false,
    };
    g_mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_mqtt_client);

    ESP_LOGI(TAG, "OneNET: pub topic = %s", pub_topic);
}

static void onenet_publish(float temp, float hum)
{
    if (!g_mqtt_client) return;

    char topic[128];
    snprintf(topic, sizeof(topic), "$sys/%s/%s/thing/property/post",
             ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    time_t now;
    time(&now);
    long long ms = (long long)now * 1000;

    char json[256];
    snprintf(json, sizeof(json),
             "{"
             "\"id\":\"123\","
             "\"version\":\"1.0\","
             "\"params\":{"
                 "\"Temp\":{\"value\":%.1f,\"time\":%lld},"
                 "\"Hum\":{\"value\":%.1f,\"time\":%lld}"
             "}"
             "}",
             temp, ms, hum, ms);

    int msg_id = esp_mqtt_client_publish(g_mqtt_client, topic, json, 0, 1, 0);
    ESP_LOGI(TAG, "OneNET publish: msg_id=%d %s", msg_id, json);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (g_wifi_retry < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            g_wifi_retry++;
            ESP_LOGW(TAG, "WiFi retry %d/%d", g_wifi_retry, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "WiFi connect failed after %d retries", WIFI_MAX_RETRY);
        }
        g_wifi_connected = 0;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        g_wifi_retry = 0;
        g_wifi_connected = 1;
        xEventGroupSetBits(g_evt, WIFI_CONNECTED_BIT);
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "ntp.aliyun.com");
        esp_sntp_set_time_sync_notification_cb(time_sync_cb);
        esp_sntp_init();
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA connecting to \"%s\"...", WIFI_SSID);
}

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2,
                                         UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART1: TX=GPIO%d RX=GPIO%d %dbps",
             UART_TX_PIN, UART_RX_PIN, UART_BAUD);
}

static void send_cmd(const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uart_write_bytes(UART_PORT, buf, len);
    ESP_LOGI(TAG, "TX: %s", buf);
}

typedef struct {
    float temperature;
    float humidity;
    float avg_temp;
    float avg_hum;
} sensor_t;

static sensor_t g_sensor = {25.5f, 60.0f, 25.5f, 60.0f};
static uint8_t  g_sensor_updated = 0;

static int parse_line(const char *line, sensor_t *s)
{
    float t, h, at, ah;
    int n = sscanf(line, "T:%f,H:%f,AVGT:%f,AVGH:%f", &t, &h, &at, &ah);
    if (n >= 2) {
        s->temperature = t;
        s->humidity    = h;
        s->avg_temp    = (n >= 4) ? at : t;
        s->avg_hum     = (n >= 4) ? ah : h;
        return 1;
    }
    return 0;
}

static void rx_task(void *arg)
{
    uint8_t buf[UART_BUF_SIZE];
    static char line[UART_BUF_SIZE];
    static int  pos = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(200));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            if (buf[i] == '\n') {
                if (pos > 0) {
                    line[pos] = '\0';
                    ESP_LOGI(TAG, "RX: %s", line);

                    if (parse_line(line, &g_sensor)) {
                        g_sensor_updated = 1;
                        ESP_LOGI(TAG, "T=%.1fC H=%.1f%% AVGT=%.1f AVGH=%.1f",
                                 g_sensor.temperature, g_sensor.humidity,
                                 g_sensor.avg_temp, g_sensor.avg_hum);
                    }
                    pos = 0;
                }
            } else if (buf[i] != '\r' && pos < (int)sizeof(line) - 1) {
                line[pos++] = buf[i];
            }
        }
    }
}

static void cmd_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(10000));
    TickType_t last_pub = xTaskGetTickCount();

    while (1) {
        if (g_sensor_updated) {
            g_sensor_updated = 0;

            float temp = g_sensor.temperature;
            float hum  = g_sensor.humidity;

            onenet_publish(temp, hum);
            last_pub = xTaskGetTickCount();

            if (temp > 26.0f || hum > 65.0f) {
                int duty = (int)((temp - 25.0f) * 20.0f);
                if (duty < 25) duty = 25;
                if (duty > 100) duty = 100;
                send_cmd("F:%d\n", duty);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }

        if (xTaskGetTickCount() - last_pub >= pdMS_TO_TICKS(5000)) {
            onenet_publish(g_sensor.temperature, g_sensor.humidity);
            last_pub = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void console_task(void *arg)
{
    char line[128];

    ESP_LOGI(TAG, "Console ready. Commands:");
    ESP_LOGI(TAG, "  pub <temp> <hum>  - publish to OneNET");
    ESP_LOGI(TAG, "  fan <duty>        - send fan duty to STM32");
    ESP_LOGI(TAG, "  servo <angle>     - send servo angle to STM32");
    ESP_LOGI(TAG, "  help              - show this");

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (line[0] < 'a' || line[0] > 'z') continue;

        float a, b;
        if (sscanf(line, "pub %f %f", &a, &b) == 2) {
            onenet_publish(a, b);
            ESP_LOGI(TAG, "Manual publish: Temp=%.1f Hum=%.1f", a, b);
        } else if (sscanf(line, "fan %f", &a) == 1) {
            int duty = (int)a;
            if (duty < 0) duty = 0;
            if (duty > 100) duty = 100;
            send_cmd("F:%d\n", duty);
            ESP_LOGI(TAG, "Fan duty: %d%%", duty);
        } else if (sscanf(line, "servo %f", &a) == 1) {
            int angle = (int)a;
            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;
            send_cmd("S:%d\n", angle);
            ESP_LOGI(TAG, "Servo angle: %d", angle);
        } else if (strncmp(line, "help", 4) == 0) {
            ESP_LOGI(TAG, "Commands: pub <t> <h> | fan <0-100> | servo <0-180> | help");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===== STM32-ESP32 OneNET Bridge =====");

    g_evt = xEventGroupCreate();
    wifi_init_sta();
    uart_init();

    xTaskCreate(rx_task,     "uart_rx",  4096, NULL, 10, NULL);
    xTaskCreate(cmd_task,    "cmd",      4096, NULL, 5,  NULL);
    xTaskCreate(console_task,"console",  4096, NULL, 3,  NULL);

    xEventGroupWaitBits(g_evt, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                        pdMS_TO_TICKS(60000));
    mqtt_init();
}
