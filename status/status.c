#include "status.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
//
#include <driver/gpio.h>

static gpio_num_t status_gpio;
static TaskHandle_t status_task_handle;
static EventGroupHandle_t status_events;

void status_set(uint8_t status) { xEventGroupSetBits(status_events, status | 0x80); }

static void status_task() {
    EventBits_t bits;
    TickType_t wait_time = portMAX_DELAY;
    while (1) {
        bits = xEventGroupWaitBits(status_events, 0xff, pdTRUE, pdFALSE, wait_time);
        if (bits & 0x80) {
            switch (bits & 0x7f) {
                case STATUS_LED_OFF:
                    wait_time = portMAX_DELAY;
                    break;
                case STATUS_LED_SLOW:
                    wait_time = pdMS_TO_TICKS(1950);
                    break;
                case STATUS_LED_MID:
                    wait_time = pdMS_TO_TICKS(450);
                    break;
                case STATUS_LED_FAST:
                default:
                    wait_time = pdMS_TO_TICKS(75);
                    break;
            }
        }
        gpio_set_level(status_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(status_gpio, 1);
    }
    vTaskDelete(NULL);
}

void status_init(gpio_num_t gpio, uint8_t status) {
    status_gpio = gpio;
    ESP_ERROR_CHECK(gpio_set_direction(status_gpio, GPIO_MODE_OUTPUT));
    status_events = xEventGroupCreate();
    status_set(status);
    xTaskCreate(status_task, "status_task", 2048, NULL, 0, &status_task_handle);
}
