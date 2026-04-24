#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
//Dioni Hernandez 2024-0985


//-----Define----------
#define ADC_CHANNEL ADC1_CHANNEL_6   // GPIO34
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define ADC_WIDTH   ADC_WIDTH_BIT_12
#define SAMPLES     1000

#define WIFI_SSID      "Campus_ITLA"
#define MQTT_BROKER    "mqtt://broker.hivemq.com"
#define MQTT_PORT      1883
#define MQTT_TOPIC     "tengo/hambre/pichardo"

static esp_mqtt_client_handle_t client;

void init_network() {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .threshold.authmode = WIFI_AUTH_OPEN } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,
        .broker.address.port = MQTT_PORT,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}


//pongame 42 pichardo

void app_main(void)
{
    init_network(); 

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN,
        ADC_WIDTH,
        1100,
        &adc_chars
    );

    printf("Iniciando medición RMS...\n");

    while (1)
    {
        uint32_t raw;
        float voltage;
        float sum = 0;
        float offset = 0;

        // 🔹 Calcular offset 
        for (int i = 0; i < SAMPLES; i++)
        {
            raw = adc1_get_raw(ADC_CHANNEL);
            voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0;
            offset += voltage;

            vTaskDelay(pdMS_TO_TICKS(1));  
        }

        offset = offset / SAMPLES;

        // 🔹 Calcular RMS 
        for (int i = 0; i < SAMPLES; i++)
        {
            raw = adc1_get_raw(ADC_CHANNEL);
            voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0;

            float v = voltage - offset;
            sum += v * v;

            vTaskDelay(pdMS_TO_TICKS(1));  
        }

        float vrms = sqrt(sum / SAMPLES);

        printf("Offset: %.3f V | Vrms: %.3f V\n", offset, vrms);

        // 🔹 Publicar a MQTT 
        char payload[64];
        sprintf(payload, "{\"vrms\": %.3f}", vrms);
        if (client != NULL) {
            esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}