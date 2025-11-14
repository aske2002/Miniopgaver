#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define MAIN_GPIO 8
#define MOTOR_GPIO 14
#define HEAT_ELEMENT 20
#define SWITCH_GPIO 2
#define TEMP_SENSOR_GPIO 6

static const char *TAG = "GPIO_APP";


// gpio_config_t out_conf = {
//     .pin_bit_mask = (1ULL << MAIN_GPIO) | (1ULL << MOTOR_GPIO) | (1ULL << HEAT_ELEMENT),
//     .mode = GPIO_MODE_OUTPUT,
//     .pull_up_en = GPIO_PULLUP_DISABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     .intr_type = GPIO_INTR_DISABLE};

// gpio_config(&out_conf);

// gpio_config_t in_conf = {
//     .pin_bit_mask = (1ULL << SWITCH_GPIO) | (1ULL << TEMP_SENSOR_GPIO),
//     .mode = GPIO_MODE_INPUT,
//     .pull_up_en = GPIO_PULLUP_ENABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     .intr_type = GPIO_INTR_DISABLE};
// gpio_config(&in_conf);
// ESP_LOGI(TAG, "Starting loop loop on GPIO %d");

// while (1) {
//     gpio_set_level(MAIN_GPIO, 1);
//     ESP_LOGI(TAG, "LED ON");

//     // Turn motor on and off 10 times
//     for (size_t i = 0; i < 20; i++)
//     {
//         gpio_set_level(MOTOR_GPIO, 1);
//         ESP_LOGI(TAG, "MOTOR ON");
//         vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
//         gpio_set_level(MOTOR_GPIO, 0);
//         ESP_LOGI(TAG, "MOTOR OFF");
//         vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
//     }

//     // Turn heating element on and off 10 times
//     for (size_t i = 0; i < 20; i++)
//     {
//         gpio_set_level(HEAT_ELEMENT, 1);
//         ESP_LOGI(TAG, "HEAT ELEMENT ON");
//         vTaskDelay(pdMS_TO_TICKS(500));  // 0.5 second delay
//         gpio_set_level(HEAT_ELEMENT, 0);
//         ESP_LOGI(TAG, "HEAT ELEMENT OFF");
//         vTaskDelay(pdMS_TO_TICKS(500));  // 0.5 second delay
//     }

//     gpio_set_level(MAIN_GPIO, 0);
//     ESP_LOGI(TAG, "LED OFF");
//     vTaskDelay(pdMS_TO_TICKS(2000));  // 2 seconds delay

// }