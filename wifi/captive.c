#include "captive.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <esp_wifi.h>
#include <lwip/sockets.h>
#include <string.h>

#include "captive-html.h"
#include "dns.h"
#include "net.h"

#define BUFFER_SIZE 256
#define OUT_BUFFER_SIZE 100
#define STRLEN(s) (sizeof(s) / sizeof(s[0]) - 1)

static const char* TAG = "captive";
static const char content_length_key[] = "Content-Length: ";
static const char content_type_key[] = "Content-Type: ";

static int parse_hex(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static int urldecode(char* dest, int dest_len, const char* src, int src_len) {
    if (dest_len < 1 || src_len < 1) {
        return -1;
    }
    char* dest_pos = dest;
    const char* src_pos = src;
    while (dest_pos < dest + dest_len - 1 && src_pos < src + src_len) {
        switch (*src_pos) {
            case '+':
                *dest_pos = ' ';
                break;

            case '%':
                if (src_pos > src + src_len - 2) {
                    return -1;
                }
                {
                    const int c1 = parse_hex(src_pos[1]);
                    const int c2 = parse_hex(src_pos[2]);
                    if (c1 < 0 || c2 < 0) {
                        return -1;
                    }
                    *dest_pos = 16 * c1 + c2;
                    src_pos += 2;
                }
                break;

            case ' ':
            case '&':
            case '\n':
            case '\r':
            case '\t':
                goto exit;

            default:
                *dest_pos = *src_pos;
                break;
        }
        ++src_pos;
        ++dest_pos;
    }
exit:
    *dest_pos = '\0';
    return src_pos - src;
}

static void send_response(struct net_connection_t* conn, int status, const char* content, int html) {
    char buf[OUT_BUFFER_SIZE];
    char* status_str;
    switch (status) {
        case 200:
            status_str = "200 OK";
            break;

        case 400:
        default:
            status_str = "400 Bad Request";
            break;
    }
    if (!content) {
        content = status_str;
    }
    const unsigned int len = strlen(content);
    const unsigned int header_len = snprintf(buf, OUT_BUFFER_SIZE,
                                             "HTTP/1.1 %s\r\n"
                                             "Content-Type: text/%s\r\n"
                                             "Content-Length: %ud\r\n"
                                             "\r\n",
                                             status_str, html ? "html" : "plain", len);
    if (net_send_all(conn->sock, buf, header_len, TAG) < 0) {
        goto exit;
    }
    net_send_all(conn->sock, content, len, TAG);
exit:
    while (recv(conn->sock, buf, OUT_BUFFER_SIZE, MSG_DONTWAIT) > 0) {
    }
    net_close_connection(conn);
}

static void handle_client_task(void* data) {
    ESP_LOGI(TAG, "client connected");
    struct net_connection_t* conn = data;

    wifi_config_t wifi_config = {0};

    char buf[BUFFER_SIZE];
    char* pos = buf;
    char* line = buf;
    char prev = '\0';
    ssize_t line_len = 0;
    ssize_t content_len = -1;
    char* errmsg = NULL;

    struct {
        uint8_t have_content_type : 1;
        uint8_t have_content_line : 1;
        uint8_t have_password : 1;
        uint8_t reboot : 1;
        uint8_t have_ssid : 1;
    } flags = {0};

    enum state_t { START, IN_HEADER, IN_CONTENT };
    enum state_t state = START;

    while (1) {
        if (state == IN_CONTENT && pos - line >= content_len) {
            flags.have_content_line = 1;
            break;
        }
        /*
        taskYIELD();
        const ssize_t len = recv(conn->sock, pos, buf + BUFFER_SIZE - pos, MSG_DONTWAIT);  // TODO use also in other components
        if (len < 0) {
            if (errno == EAGAIN) {
                continue;
            }
            if (errno == ENOTCONN) {
                break;
            }
            ESP_LOGE(TAG, "recv failed: %s", strerror(errno));
            net_free_connection(conn);
            return;
        }
        if (len == 0) {
            continue;
        }
        */
        const ssize_t len = recv(conn->sock, pos, buf + BUFFER_SIZE - pos, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: %s", strerror(errno));
            net_free_connection(conn);
            return;
        }
        if (len == 0) {
            ESP_LOGW(TAG, "connection closed");
            break;
        }
        for (int i = 0; i < len; ++i) {
            if (*pos == '\n' && prev == '\r') {
                line_len = pos - 1 - line;
                switch (state) {
                    case START:
                        if (line_len < 5) {
                            errmsg = "Unexpected end in first line of request";
                            goto exit;
                        }
                        if (!strncasecmp(line, "GET ", 4)) {
                            send_response(conn, 200, captive_html, 1);
                            net_free_connection(conn);
                            return;
                        }
                        if (strncasecmp(line, "POST ", 5)) {
                            ESP_LOGW(TAG, "Received line %.*s", line_len, line);
                            errmsg = "Wrong method";
                            goto exit;
                        }
                        state = IN_HEADER;
                        break;

                    case IN_HEADER:
                        if (line_len == 0) {
                            if (!flags.have_content_type) {
                                errmsg = "No content type provided";
                                goto exit;
                            }
                            if (content_len < 0) {
                                errmsg = "No content length provided";
                                goto exit;
                            }
                            state = IN_CONTENT;
                            break;
                        }
                        if (!flags.have_content_type && line_len >= STRLEN(content_type_key)
                            && !strncasecmp(line, content_type_key, STRLEN(content_type_key))) {
                            /* line starts with "Content-Type: " */
                            static const char expected_content_type[] = "application/x-www-form-urlencoded";
                            if (line_len != STRLEN(content_type_key) + STRLEN(expected_content_type)
                                || strncasecmp(line + STRLEN(content_type_key), expected_content_type, STRLEN(expected_content_type))) {
                                ESP_LOGW(TAG, "Received line %.*s", line_len, line);
                                errmsg = "Wrong content type";
                                goto exit;
                            }
                            flags.have_content_type = 1;
                        }
                        if (content_len < 0 && line_len >= STRLEN(content_length_key) && !strncasecmp(line, content_length_key, STRLEN(content_length_key))) {
                            /* line starts with "Content-Length: " */
                            pos[-1] = '\0';
                            char* end;
                            content_len = strtoul(line + STRLEN(content_length_key), &end, 10);
                            if (end != pos - 1) {
                                ESP_LOGW(TAG, "Received line %.*s", line_len, line);
                                errmsg = "Could not parse content length";
                                goto exit;
                            }
                        }
                        break;

                    case IN_CONTENT:
                        break;
                }
                line = pos + 1;
            }
            prev = *pos;
            ++pos;
        }
        if (line == buf) {
            ESP_LOGW(TAG, "Received line %.*s", line_len, line);
            errmsg = "Buffer overflow";
            goto exit;
        }
        memmove(buf, line, pos - line);
        pos = buf + (pos - line);
        line = buf;
    }

    if (!flags.have_content_line) {
        errmsg = "Did not receive the full content line";
        goto exit;
    }

    char* key = line;
    for (int i = 0; i < content_len; ++i) {
        if (line[i] == '=') {
            const ssize_t key_len = line + i - key;
            int c = -1;
            if (key_len == 4 && !strncasecmp(key, "ssid", 4)) {
                c = urldecode((char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), line + i + 1, content_len - i - 1);
                if (c >= 0) {
                    flags.have_ssid = 1;
                }
            } else if (key_len == 8 && !strncasecmp(key, "password", 8)) {
                c = urldecode((char*)wifi_config.sta.password, sizeof(wifi_config.sta.password), line + i + 1, content_len - i - 1);
                if (c >= 0) {
                    flags.have_password = 1;
                }
            } else if (key_len == 6 && !strncasecmp(key, "reboot", 6)) {
                flags.reboot = 1;
                goto exit;
            }
            if (c < 0) {
                ESP_LOGW(TAG, "Received line %.*s", line_len, line);
                errmsg = "Bad parameters";
                goto exit;
            }
            i += c + 1;
            key = line + i + 1;
        }
    }

    if (!flags.have_ssid) {
        ESP_LOGW(TAG, "Received line %.*s", line_len, line);
        errmsg = "No ssid provided";
        goto exit;
    }
    if (!flags.have_password) {
        ESP_LOGW(TAG, "Received line %.*s", line_len, line);
        errmsg = "No password provided";
        goto exit;
    }

exit:
    if (errmsg) {
        ESP_LOGW(TAG, "HTTP error: %s", errmsg);
        send_response(conn, 400, errmsg, 0);
    } else {
        if (flags.reboot) {
            send_response(conn, 200, "Restarting...", 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            send_response(conn, 200, "Connecting...", 0);
            captive_stop();
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
            ESP_ERROR_CHECK(esp_wifi_start());
        }
    }
    net_free_connection(conn);
}

static struct net_server_t server = {
    .port = 80,
    .max_connections = 1,
    .client_task = handle_client_task,
    .client_task_stack_depth = 4096,
    .client_task_priority = 2,
};

void captive_start() {
    net_start_server(&server, 1);
    dns_start();
}

void captive_stop() {
    dns_stop();
    net_stop_server(&server);
}
