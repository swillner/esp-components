#include "ota.h"

#include <freertos/FreeRTOS.h>
//
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <lwip/sockets.h>

#include "net.h"

#define BUFFER_SIZE 1024
static const char* TAG = "ota";

static void handle_client_task(void* data) {
    ESP_LOGI(TAG, "client connected");
    struct net_connection_t* conn = data;
    unsigned char buf[BUFFER_SIZE];
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(NULL);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        goto exit_task;
    }

    while (1) {
        ssize_t len = recv(conn->sock, buf, BUFFER_SIZE, 0);
        ESP_LOGE(TAG, "received %d bytes", len);
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: %s", strerror(errno));
            goto exit_error;
        }
        if (len == 0) {
            net_close_connection(conn);
            ESP_LOGI(TAG, "all data received");
            err = esp_ota_end(update_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
                goto exit_error;
            }
            err = esp_ota_set_boot_partition(update_partition);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
                goto exit_error;
            }
            ESP_LOGI(TAG, "restarting");
            esp_restart();
            goto exit_task;
        }
        err = esp_ota_write(update_handle, buf, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            goto exit_error;
        }
    }

exit_error:
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    esp_ota_abort(update_handle);
#endif

exit_task:
    net_free_connection(conn);
}

static struct net_server_t server = {
    .port = 25431,
    .max_connections = 1,
    .client_task = handle_client_task,
    .client_task_stack_depth = 4096,
    .client_task_priority = 255,
    .flags = {.own_task = 1},
};

void ota_start() { net_start_server(&server, 1); }
void ota_stop() { net_stop_server(&server); }
