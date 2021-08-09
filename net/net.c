#include "net.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <lwip/sockets.h>

#define USE_IPV4

static const char* TAG = "net";

static uint16_t last_loopback_port = 32768;
static SemaphoreHandle_t last_loopback_port_mutex;

void net_init() { last_loopback_port_mutex = xSemaphoreCreateMutex(); }

static void listen_task(void* data) {
    struct net_server_t* server = data;

    if (net_loopback_sock_new(&server->internal.loopback)) {
        ESP_LOGE(TAG, "net_loopback_sock_new failed for server port %u", server->port);
        goto exit_task;
    }

#ifdef USE_IPV4
    struct sockaddr_in dest = {
        .sin_addr = {.s_addr = htonl(INADDR_ANY)},
        .sin_family = AF_INET,
        .sin_port = htons(server->port),
    };
    server->internal.listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
#else
    struct sockaddr_in6 dest = {
        .sin6_addr = {0},
        .sin6_family = AF_INET6,
        .sin6_port = htons(server->port),
    };
    server->internal.listen_sock = socket(AF_INET6, SOCK_STREAM, IPPROTO_IPV6);
#endif

    if (server->internal.listen_sock < 0) {
        ESP_LOGE(TAG, "socket failed for port %u: %s", server->port, strerror(errno));
        goto exit_task;
    }

    if (net_make_reusable(server->internal.listen_sock)) {
        ESP_LOGE(TAG, "net_make_reusable failed for port %u: %s", server->port, strerror(errno));
        goto exit_task;
    }

    if (bind(server->internal.listen_sock, (struct sockaddr*)&dest, sizeof(dest))) {
        ESP_LOGE(TAG, "bind failed for port %u: %s", server->port, strerror(errno));
        close(server->internal.listen_sock);
        goto exit_task;
    }

    if (listen(server->internal.listen_sock, server->max_connections)) {
        ESP_LOGE(TAG, "listen failed for port %u: %s", server->port, strerror(errno));
        close(server->internal.listen_sock);
        goto exit_task;
    }

    ESP_LOGI(TAG, "waiting for clients on port %u", server->port);
    while (1) {
        if (net_loopback_select(&server->internal.loopback, server->internal.listen_sock)) {
            ESP_LOGI(TAG, "received shutdown signal for server on port %u", server->port);
            break;
        }
        int sock = accept(server->internal.listen_sock, NULL, NULL);
        if (sock < 0) {
            ESP_LOGE(TAG, "accept failed for port %u: %s", server->port, strerror(errno));
            break;
        }
        struct net_connection_t* conn = malloc(sizeof(struct net_connection_t));
        conn->sock = sock;
        conn->flags.own_task = server->flags.own_task || (server->max_connections != 1);
        conn->flags.shutdown_server = 0;
        if (conn->flags.own_task) {
            xTaskCreate(server->client_task, "net_client_task", server->client_task_stack_depth, conn, server->client_task_priority, NULL);
        } else {
            server->client_task(conn);
            if (conn->flags.shutdown_server) {
                break;
            }
        }
    }

exit_task:
    if (server->internal.listen_sock >= 0) {
        shutdown(server->internal.listen_sock, SHUT_RDWR);
        close(server->internal.listen_sock);
        server->internal.listen_sock = -1;
        ESP_LOGI(TAG, "stopped server on port %u", server->port);
    }
    net_loopback_sock_close(&server->internal.loopback);
    server->internal.server_task_handle = NULL;
    vTaskDelete(NULL);
}

ssize_t net_recv_all(int sock, void* buf, ssize_t len, const char* tag, int* disconnected) {
    *disconnected = 1;
    ssize_t received = 0;
    while (received < len) {
        ssize_t c = recv(sock, buf + received, len - received, 0);
        if (c < 0) {
            ESP_LOGE(tag, "recv failed: %s", strerror(errno));
            return -1;
        }
        if (c == 0) {
            ESP_LOGI(tag, "client disconnected");
            return received;
        }
        received += c;
    }
    *disconnected = 0;
    return received;
}

ssize_t net_send_all(int sock, const void* buf, ssize_t len, const char* tag) {
    ssize_t sent = 0;
    while (sent < len) {
        ssize_t c = send(sock, buf + sent, len - sent, 0);
        if (c <= 0) {
            ESP_LOGE(tag, "send failed: %s", strerror(errno));
            return -1;
        }
        sent += c;
    }
    return sent;
}

void net_start_server(struct net_server_t* server, UBaseType_t server_task_priority) {
    server->internal.listen_sock = -1;
    xTaskCreate(listen_task, "net_listen_task",
                server->max_connections != 1 ? 1024 /* when creating a task per client connection */
                                             : server->client_task_stack_depth /* otherwise use user-provided stack depth */,
                server, server_task_priority, &server->internal.server_task_handle);
}

int net_make_reusable(int sock) {
    const int val = 1;
    return setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&val, sizeof(val));
}

void net_stop_server(struct net_server_t* server) {
    if (server->internal.listen_sock >= 0) {
        if (net_loopback_sock_send_byte(&server->internal.loopback, NET_CONTROL_SHUTDOWN) < 0) {
            ESP_LOGE(TAG, "net_loopback_sock_send_byte failed for port %u: %s", server->internal.loopback.port, strerror(errno));
        }
    }
}

void net_close_connection(struct net_connection_t* conn) {
    if (conn->sock >= 0) {
        shutdown(conn->sock, SHUT_RDWR);
        close(conn->sock);
        conn->sock = -1;
    }
}

void net_free_connection(struct net_connection_t* conn) {
    net_close_connection(conn);
    int own_task = conn->flags.own_task;
    free(conn);
    if (own_task) {
        vTaskDelete(NULL);
    }
}

int net_loopback_select(struct net_loopback_sock_t* sock, int other_sock) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(other_sock, &set);
    FD_SET(sock->recv, &set);

    if (select((other_sock > sock->recv ? other_sock : sock->recv) + 1, &set, NULL, NULL, NULL) < 0) {
        return -1;
    }

    if (FD_ISSET(sock->recv, &set)) {
        uint8_t res;
        if (net_loopback_sock_recv(sock, &res, sizeof(res)) < 0) {
            ESP_LOGE(TAG, "net_loopback_sock_recv failed for port %u: %s", sock->port, strerror(errno));
            return -1;
        }
        return res;
    }

    return 0;
}

int net_loopback_sock_new(struct net_loopback_sock_t* sock) {
    xSemaphoreTake(last_loopback_port_mutex, portMAX_DELAY);
    sock->port = last_loopback_port++;
    xSemaphoreGive(last_loopback_port_mutex);

    struct sockaddr_in dest = {
        .sin_addr = {.s_addr = htonl(INADDR_LOOPBACK)},
        .sin_family = AF_INET,
        .sin_port = htons(sock->port),
    };

    sock->recv = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    sock->send = -1;

    if (sock->recv < 0) {
        ESP_LOGE(TAG, "loopback socket failed for port %u: %s", sock->port, strerror(errno));
        return -1;
    }

    if (bind(sock->recv, (struct sockaddr*)&dest, sizeof(dest))) {
        ESP_LOGE(TAG, "loopback bind failed for port %u: %s", sock->port, strerror(errno));
        net_loopback_sock_close(sock);
        return -1;
    }

    sock->send = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sock->send < 0) {
        ESP_LOGE(TAG, "loopback socket failed for port %u: %s", sock->port, strerror(errno));
        net_loopback_sock_close(sock);
        return -1;
    }

    return 0;
}

int net_loopback_sock_recv(struct net_loopback_sock_t* sock, void* data, int len) { return recvfrom(sock->recv, data, len, 0, NULL, NULL); }

int net_loopback_sock_send(struct net_loopback_sock_t* sock, const void* data, int len) {
    struct sockaddr_in dest = {
        .sin_addr = {.s_addr = htonl(INADDR_LOOPBACK)},
        .sin_family = AF_INET,
        .sin_port = htons(sock->port),
    };
    return sendto(sock->send, data, len, 0, (struct sockaddr*)&dest, sizeof(dest));
}

inline int net_loopback_sock_send_byte(struct net_loopback_sock_t* sock, uint8_t val) { return net_loopback_sock_send(sock, &val, sizeof(val)); }

void net_loopback_sock_close(struct net_loopback_sock_t* sock) {
    if (sock->recv >= 0) {
        shutdown(sock->recv, SHUT_RDWR);
        close(sock->recv);
        sock->recv = -1;
    }
    if (sock->send >= 0) {
        shutdown(sock->send, SHUT_RDWR);
        close(sock->send);
        sock->send = -1;
    }
}
