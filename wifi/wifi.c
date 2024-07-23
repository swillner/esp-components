#include "wifi.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <freertos/timers.h>
//
#include <argtable3/argtable3.h>
#include <esp_console.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>

#include "captive.h"
#include "status.h"

#define AP_SSID "esp_config"
#define AP_PASS "correcthorsestaple"

#define WIFI_EVENT_CONNECTED 0x01
#define WIFI_EVENT_IS_AP 0x02

#define CONNECT_TIMEOUT 10000
#define CONNECT_MAX_TRIES 10

static const char* TAG = "wifi";

static EventGroupHandle_t wifi_events;
static TimerHandle_t timeout_timer;

static struct {
    struct arg_str* ssid;
    struct arg_str* password;
    struct arg_end* end;
} wifi_cmd_join_args;

static int wifi_cmd_join(int argc, char** argv) {
    int nerrors = arg_parse(argc, argv, (void**)&wifi_cmd_join_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_cmd_join_args.end, argv[0]);
        return 1;
    }

    const char* ssid = wifi_cmd_join_args.ssid->sval[0];
    const char* pass = wifi_cmd_join_args.password->sval[0];

    ESP_LOGI(TAG, "connecting to '%s'", ssid);

    wifi_config_t wifi_config = {0};

    strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass) {
        strlcpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    return 0;
}

static void wifi_timeout_handler(TimerHandle_t timer) {
    ESP_LOGW(TAG, "connection timed out, starting access point");
    wifi_start_access_point();
    captive_start();
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int event_id, void* event_data) {
    static int retry_cnt = 0;

    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                status_set(STATUS_LED_MID);
                xTimerReset(timeout_timer, portMAX_DELAY);
                esp_wifi_connect();
                retry_cnt = 0;
                ESP_LOGI(TAG, "trying to connect to access point");
                break;

            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t* event = event_data;
                ESP_LOGI(TAG, "connected to %.*s", event->ssid_len, event->ssid);
            } break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_connected_t* event = event_data;
                xEventGroupClearBits(wifi_events, WIFI_EVENT_CONNECTED);
                status_set(STATUS_LED_MID);
                ESP_LOGI(TAG, "disconnected from %.*s", event->ssid_len, event->ssid);
                if (!wifi_is_access_point()) {
                    if (retry_cnt >= CONNECT_MAX_TRIES) {
                        xTimerStop(timeout_timer, portMAX_DELAY);
                        wifi_timeout_handler(timeout_timer);
                    } else {
                        xTimerReset(timeout_timer, portMAX_DELAY);
                        ESP_ERROR_CHECK(esp_wifi_connect());
                    }
                    ++retry_cnt;
                }
            } break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = event_data;
                ESP_LOGI(TAG, "client " MACSTR " connected", MAC2STR(event->mac));
            } break;

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = event_data;
                ESP_LOGI(TAG, "client " MACSTR " disconnected", MAC2STR(event->mac));
            } break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                retry_cnt = 0;
                xTimerStop(timeout_timer, portMAX_DELAY);
                xEventGroupSetBits(wifi_events, WIFI_EVENT_CONNECTED);
                status_set(STATUS_LED_OFF);
                ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
                ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
            } break;
        }
    }
}

void wifi_console_register() {
    wifi_cmd_join_args.ssid = arg_str1(NULL, NULL, "<ssid>", "ssid");
    wifi_cmd_join_args.password = arg_str0(NULL, NULL, "<pass>", "password");
    wifi_cmd_join_args.end = arg_end(2);
    const esp_console_cmd_t wifi_cmd_join_reg = {.command = "join", .help = "join wifi", .hint = NULL, .func = &wifi_cmd_join, .argtable = &wifi_cmd_join_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_cmd_join_reg));
}

uint32_t wifi_get_local_ip() {
    tcpip_adapter_if_t tcpip_if = wifi_is_access_point() ? TCPIP_ADAPTER_IF_AP : TCPIP_ADAPTER_IF_STA;
    tcpip_adapter_ip_info_t ip_info;
    tcpip_adapter_get_ip_info(tcpip_if, &ip_info);
    return ip_info.ip.addr;
}

void wifi_init() {
    wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    esp_netif_create_default_wifi_sta();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    timeout_timer = xTimerCreate("wifi_timeout_timer", pdMS_TO_TICKS(CONNECT_TIMEOUT), pdFALSE, NULL, wifi_timeout_handler);
}

int wifi_is_access_point() { return xEventGroupGetBits(wifi_events) & WIFI_EVENT_IS_AP; }

int wifi_is_connected() { return xEventGroupGetBits(wifi_events) & WIFI_EVENT_CONNECTED; }

void wifi_connect() {
    xTimerStop(timeout_timer, portMAX_DELAY);
    xEventGroupClearBits(wifi_events, WIFI_EVENT_CONNECTED | WIFI_EVENT_IS_AP);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_start_access_point() {
    xTimerStop(timeout_timer, portMAX_DELAY);
    xEventGroupClearBits(wifi_events, WIFI_EVENT_CONNECTED);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    esp_netif_create_default_wifi_ap();
#endif

    wifi_config_t wifi_config = {
        .ap = {.ssid = AP_SSID, .ssid_len = strlen(AP_SSID), .password = AP_PASS, .max_connection = 1, .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    xEventGroupSetBits(wifi_events, WIFI_EVENT_IS_AP);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    status_set(STATUS_LED_SLOW);
    ESP_ERROR_CHECK(esp_wifi_start());
}

int wifi_wait_for_connection(TickType_t wait_time) {
    return xEventGroupWaitBits(wifi_events, WIFI_EVENT_CONNECTED, pdFALSE, pdFALSE, wait_time) & WIFI_EVENT_CONNECTED;
}
