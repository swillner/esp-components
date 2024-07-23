#include "logport.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
//
#include <esp_log.h>
#include <lwip/sockets.h>

#include "net.h"

#define PORT 517
#define BUFFER_SIZE 256
#define LOGPORT_DISCONNECTED 0x01
static const char* TAG = "logport";

static struct net_connection_t* conn = NULL;
static char buf[BUFFER_SIZE];
static SemaphoreHandle_t mutex;
static EventGroupHandle_t event_group;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)

static int IRAM_ATTR logport_vprintf(const char* args, va_list vargs) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int rc = vsnprintf(buf, BUFFER_SIZE, args, vargs);
    if (rc > 0 && conn && net_send_all(conn->sock, buf, rc, TAG) < 0) {
        xEventGroupSetBits(event_group, LOGPORT_DISCONNECTED);
    }
    xSemaphoreGive(mutex);
    return rc;
}

#else

static char* pos = buf;
static int IRAM_ATTR logport_putc(int c) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    *pos = c;
    if (c == '\n' || pos == buf + BUFFER_SIZE - 1) {
        if (conn && net_send_all(conn->sock, buf, pos + 1 - buf, TAG) < 0) {
            xEventGroupSetBits(event_group, LOGPORT_DISCONNECTED);
        }
        pos = buf;
    } else {
        ++pos;
    }
    xSemaphoreGive(mutex);
    return c;
}

#endif

static void handle_client_task(void* data) {
    xEventGroupClearBits(event_group, LOGPORT_DISCONNECTED);

    xSemaphoreTake(mutex, portMAX_DELAY);
    conn = data;
    xSemaphoreGive(mutex);

    xEventGroupWaitBits(event_group, 0xff, pdTRUE, pdFALSE, portMAX_DELAY);

    xSemaphoreTake(mutex, portMAX_DELAY);
    conn = NULL;
    xSemaphoreGive(mutex);

    net_free_connection(data);
}

static struct net_server_t server = {
    .port = PORT,
    .max_connections = 1,
    .client_task = handle_client_task,
    .client_task_stack_depth = 4096,
    .client_task_priority = 2,
};

void logport_start() {
    event_group = xEventGroupCreate();
    mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "switching logging output to port %u", PORT);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    esp_log_set_vprintf(logport_vprintf);
#else
    esp_log_set_putchar(logport_putc);
#endif
    net_start_server(&server, 1);
}

void logport_stop() { net_stop_server(&server); }
