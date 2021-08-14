#ifndef NET_H
#define NET_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

enum {
    NET_CONTROL_SHUTDOWN = 1,
};

struct net_connection_t {
    int sock;
    void* client_data;
    struct {
        uint8_t own_task : 1;        /* is set if client task has its own task (i.e. max_connections != 1) */
        uint8_t shutdown_server : 1; /* can be set by client task to shutdown server (only when max_connections == 1) */
    } flags;
};

struct net_loopback_sock_t {
    uint16_t port;
    int recv;
    int send;
};

struct net_server_t {
    uint16_t port;
    uint16_t max_connections;
    TaskFunction_t client_task;
    configSTACK_DEPTH_TYPE client_task_stack_depth;
    UBaseType_t client_task_priority;
    void* client_data;

    struct {
        uint8_t own_task : 1;
    } flags;

    struct {
        TaskHandle_t server_task_handle;
        int listen_sock;
        struct net_loopback_sock_t loopback;
    } internal; /* set by net_start_server */
};

void net_init();

ssize_t net_recv_all(int sock, void* buf, ssize_t len, const char* tag, int* disconnected);
ssize_t net_send_all(int sock, const void* buf, ssize_t len, const char* tag);
void net_close_connection(struct net_connection_t* connection);
void net_free_connection(struct net_connection_t* connection);
void net_start_server(struct net_server_t* server, UBaseType_t server_task_priority);
void net_stop_server(struct net_server_t* server);
int net_make_reusable(int sock);

int net_loopback_select(struct net_loopback_sock_t* sock, int other_sock);
int net_loopback_sock_new(struct net_loopback_sock_t* sock);
int net_loopback_sock_recv(struct net_loopback_sock_t* sock, void* data, int len);
int net_loopback_sock_send(struct net_loopback_sock_t* sock, const void* data, int len);
int net_loopback_sock_send_byte(struct net_loopback_sock_t* sock, uint8_t val);
void net_loopback_sock_close(struct net_loopback_sock_t* sock);

#endif
