#include "telnet.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <lwip/sockets.h>
#include <string.h>

#ifdef TELNET_UART_SUPPORT
#include <driver/uart.h>
#endif

#include "net.h"

// #define TELNET_DEBUG
#ifdef TELNET_DEBUG
#define TELCMDS
#define TELOPTS
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#pragma GCC diagnostic ignored "-Wtype-limits"
#include "arpa/telnet.h"
#pragma GCC diagnostic pop

#define TELOPT_COM_PORT 44
#define TELNET_COM_PORT_SET_BAUDRATE 1
#define TELNET_COM_PORT_SET_DATASIZE 2
#define TELNET_COM_PORT_SET_PARITY 3
#define TELNET_COM_PORT_SET_STOPSIZE 4
#define TELNET_COM_PORT_SET_CONTROL 5
#define TELNET_COM_PORT_PURGE_DATA 12

#define ARRAYLEN(a) (sizeof(a) / sizeof(a[0]))

#define BUFFER_SIZE 256
#define MAX_LINE_LENGTH 256
static const unsigned char prompt[] = {'>', ' ', IAC, GA};
static const char* TAG = "telnet";

struct telnet_option_t {
    uint8_t local_enabled : 1;
    uint8_t local_sent : 1;
    uint8_t local_supported : 1;
    uint8_t remote_enabled : 1;
    uint8_t remote_sent : 1;
    uint8_t remote_supported : 1;
};

struct telnet_connection_t {
    int sock;
    struct telnet_option_t options[256];
    unsigned char inbuf[MAX_LINE_LENGTH];
    unsigned char* inpos;
    unsigned char outbuf[MAX_LINE_LENGTH];
    unsigned char* outpos;
    SemaphoreHandle_t mutex;
};

#ifdef TELNET_UART_SUPPORT
static struct telnet_config_t {
    uart_port_t uart;
    gpio_num_t dtr_pin;
    gpio_num_t rts_pin;
    uint8_t uart_enabled : 1;
    QueueHandle_t uart_queue;
    TaskHandle_t uart_event_task;
} config;
#endif

static inline int get_prompt_len(struct telnet_connection_t* conn) { return ARRAYLEN(prompt) - (conn->options[TELOPT_SGA].local_enabled ? 2 : 0); }

static int send_outbuf(struct telnet_connection_t* conn) {
    const int len = conn->outpos - conn->outbuf;
    if (len > 0) {
#ifdef TELNET_DEBUG
        ESP_LOGI(TAG, "send %u bytes", len);
#endif
        if (net_send_all(conn->sock, conn->outbuf, len, TAG) < 0) {
            return 1;
        }
        conn->outpos = conn->outbuf;
    }
    return 0;
}

static int add_to_outbuf(struct telnet_connection_t* conn, const unsigned char* buf, int len) {
    xSemaphoreTake(conn->mutex, portMAX_DELAY);
    int left = conn->outbuf + ARRAYLEN(conn->outbuf) - conn->outpos;
    while (1) {
        int l = left < len ? left : len;
        memcpy(conn->outpos, buf, l);
        conn->outpos += l;
        len -= l;
        if (len == 0) {
            xSemaphoreGive(conn->mutex);
            return 0;
        }
        if (send_outbuf(conn)) {
            xSemaphoreGive(conn->mutex);
            return 1;
        }
        left = ARRAYLEN(conn->outbuf);
    }
}

static int add_to_outbuf_escaped(struct telnet_connection_t* conn, const unsigned char* buf, int len) {
    xSemaphoreTake(conn->mutex, portMAX_DELAY);
    for (int i = 0; i < len; ++i) {
        if (conn->outpos >= conn->outbuf + ARRAYLEN(conn->outbuf) - 1) {
            if (send_outbuf(conn)) {
                xSemaphoreGive(conn->mutex);
                return 1;
            }
        }
        *conn->outpos = buf[i];
        ++conn->outpos;
        if (buf[i] == IAC) {
            *conn->outpos = IAC;
            ++conn->outpos;
        }
    }
    xSemaphoreGive(conn->mutex);
    return 0;
}

static int process_line(struct telnet_connection_t* conn) {
#ifdef TELNET_UART_SUPPORT
    if (conn->options[TELOPT_COM_PORT].remote_enabled) {
        ESP_LOGI(TAG, "received line of length %u", conn->inpos - conn->inbuf);
        int res = uart_write_bytes(config.uart, (const char*)conn->inbuf, conn->inpos - conn->inbuf);
        if (res < 0) {
            return 1;
        }
        conn->inpos = conn->inbuf + (conn->inbuf - conn->inpos + res);
        return 0;
    }
#endif
#ifdef TELNET_DEBUG
    if (conn->options[TELOPT_BINARY].local_enabled) {
        ESP_LOGI(TAG, "received line of length %u", conn->inpos - conn->inbuf);
    } else {
        ESP_LOGI(TAG, "received line %.*s", conn->inpos - conn->inbuf, conn->inbuf);
    }
#endif
    conn->inpos = conn->inbuf;
    return add_to_outbuf(conn, prompt, get_prompt_len(conn));
}

static inline void add_to_line(struct telnet_connection_t* conn, unsigned char c) {
    if (conn->inbuf + ARRAYLEN(conn->inbuf) - conn->inpos < 1) {
        if (conn->options[TELOPT_BINARY].local_enabled) {
            process_line(conn);
        } else {
            --conn->inpos;
        }
    }
    conn->inpos[0] = c;
    ++conn->inpos;
}

#ifdef TELNET_DEBUG
static void log_command(const char* prefix, unsigned char command, unsigned char option) {
    switch (command) {
        case DO:
        case DONT:
        case WILL:
        case WONT:
            if (TELOPT_OK(option)) {
                ESP_LOGI(TAG, "%s IAC %s %s", prefix, TELCMD(command), TELOPT(option));
            } else {
                ESP_LOGI(TAG, "%s IAC %s %u", prefix, TELCMD(command), option);
            }
            break;

        default:
            if (command >= TELCMD_FIRST) {
                ESP_LOGI(TAG, "%s IAC %s", prefix, TELCMD(command));
            } else {
                ESP_LOGI(TAG, "%s IAC %u", prefix, command);
            }
            break;
    }
}
#else
#define log_command(prefix, command, option)
#endif

#ifdef TELNET_UART_SUPPORT
static void uart_event_task(void* data) {
    struct telnet_connection_t* conn = data;
    uart_event_t event;
    unsigned char buf[128];

    while (1) {
        if (xQueueReceive(config.uart_queue, (void*)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
#ifdef TELNET_DEBUG
                    ESP_LOGI(TAG, "uart received %d bytes", event.size);
#endif
                    while (event.size > 0) {
                        int l = event.size > ARRAYLEN(buf) ? ARRAYLEN(buf) : event.size;
                        uart_read_bytes(config.uart, buf, l, portMAX_DELAY);
                        add_to_outbuf_escaped(conn, buf, l);
                        event.size -= l;
                    }
                    xSemaphoreTake(conn->mutex, portMAX_DELAY);
                    send_outbuf(conn);
                    xSemaphoreGive(conn->mutex);
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "uart fifo overflow");
                    uart_flush_input(config.uart);
                    xQueueReset(config.uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "uart buffer full");
                    uart_flush_input(config.uart);
                    xQueueReset(config.uart_queue);
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "uart parity error");
                    break;

                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "uart frame error");
                    break;

                default:
                    ESP_LOGW(TAG, "uart unknown event %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}
#endif

#ifdef TELNET_UART_SUPPORT
static int handle_com_port_option(struct telnet_connection_t* conn, unsigned char option, uint32_t value) {
    switch (option) {
        case TELNET_COM_PORT_SET_BAUDRATE:
            ESP_LOGI(TAG, "COM-PORT set-baudrate %u", value);
            if (value) {
                uart_set_baudrate(config.uart, value);
            }
            if (uart_get_baudrate(config.uart, &value) == ESP_OK) {
                return add_to_outbuf(
                    conn, (const unsigned char[]){IAC, SB, TELOPT_COM_PORT, TELNET_COM_PORT_SET_BAUDRATE, value >> 24, value >> 16, value >> 8, value, IAC, SE},
                    10);
            }
            break;

        case TELNET_COM_PORT_SET_DATASIZE:
            ESP_LOGI(TAG, "COM-PORT set-datasize %u", value);
            switch (value) {
                case 0: /* Request Current Data Bit Size */
                    break;
                case 5: /* 5 */
                    uart_set_word_length(config.uart, UART_DATA_5_BITS);
                    break;
                case 6: /* 6 */
                    uart_set_word_length(config.uart, UART_DATA_6_BITS);
                    break;
                case 7: /* 7 */
                    uart_set_word_length(config.uart, UART_DATA_7_BITS);
                    break;
                case 8: /* 8 */
                    uart_set_word_length(config.uart, UART_DATA_8_BITS);
                    break;
            }
            {
                uart_word_length_t word_length;
                if (uart_get_word_length(config.uart, &word_length) == ESP_OK) {
                    uint8_t val = 0;
                    switch (word_length) {
                        case UART_DATA_5_BITS:
                            val = 5;
                            break;
                        case UART_DATA_6_BITS:
                            val = 6;
                            break;
                        case UART_DATA_7_BITS:
                            val = 7;
                            break;
                        case UART_DATA_8_BITS:
                            val = 8;
                            break;
                        case UART_DATA_BITS_MAX:
                            break;
                    }
                    if (val) {
                        return add_to_outbuf(conn, (const unsigned char[]){IAC, SB, TELOPT_COM_PORT, TELNET_COM_PORT_SET_PARITY, val, IAC, SE}, 7);
                    }
                }
            }
            break;

        case TELNET_COM_PORT_SET_PARITY:
            ESP_LOGI(TAG, "COM-PORT set-parity %u", value);
            switch (value) {
                case 0: /* Request Current Data Size */
                    break;
                case 1: /* NONE */
                    uart_set_parity(config.uart, UART_PARITY_DISABLE);
                    break;
                case 2: /* ODD */
                    uart_set_parity(config.uart, UART_PARITY_ODD);
                    break;
                case 3: /* EVEN */
                    uart_set_parity(config.uart, UART_PARITY_EVEN);
                    break;
                case 4: /* MARK */
                    break;
                case 5: /* SPACE */
                    break;
            }
            {
                uart_parity_t parity;
                if (uart_get_parity(config.uart, &parity) == ESP_OK) {
                    uint8_t val = 0;
                    switch (parity) {
                        case UART_PARITY_DISABLE:
                            val = 1;
                            break;
                        case UART_PARITY_ODD:
                            val = 2;
                            break;
                        case UART_PARITY_EVEN:
                            val = 3;
                            break;
                    }
                    if (val) {
                        return add_to_outbuf(conn, (const unsigned char[]){IAC, SB, TELOPT_COM_PORT, TELNET_COM_PORT_SET_PARITY, val, IAC, SE}, 7);
                    }
                }
            }
            break;

        case TELNET_COM_PORT_SET_STOPSIZE:
            ESP_LOGI(TAG, "COM-PORT set-stopsize %u", value);
            switch (value) {
                case 0: /* Request Current Data Size */
                    break;
                case 1: /* 1 */
                    uart_set_stop_bits(config.uart, UART_STOP_BITS_1);
                    break;
                case 2: /* 2 */
                    uart_set_stop_bits(config.uart, UART_STOP_BITS_2);
                    break;
                case 3: /* 1.5 */
                    uart_set_stop_bits(config.uart, UART_STOP_BITS_1_5);
                    break;
            }
            {
                uart_stop_bits_t stop_bits;
                if (uart_get_stop_bits(config.uart, &stop_bits) == ESP_OK) {
                    uint8_t val = 0;
                    switch (stop_bits) {
                        case UART_STOP_BITS_1:
                            val = 1;
                            break;
                        case UART_STOP_BITS_1_5:
                            val = 3;
                            break;
                        case UART_STOP_BITS_2:
                            val = 2;
                            break;
                        case UART_STOP_BITS_MAX:
                            break;
                    }
                    if (val) {
                        return add_to_outbuf(conn, (const unsigned char[]){IAC, SB, TELOPT_COM_PORT, TELNET_COM_PORT_SET_STOPSIZE, val, IAC, SE}, 7);
                    }
                }
            }
            break;

        case TELNET_COM_PORT_SET_CONTROL:
            switch (value) {
                case 0:  /* Request Com Port Flow Control Setting (outbound/both) */
                case 1:  /* Use No Flow Control (outbound/both) */
                case 2:  /* Use XON/XOFF Flow Control (outbound/both) */
                case 3:  /* Use HARDWARE Flow Control (outbound/both) */
                case 4:  /* Request BREAK State */
                case 5:  /* Set BREAK State ON */
                case 6:  /* Set BREAK State OFF */
                case 7:  /* Request DTR Signal State */
                case 10: /* Request RTS Signal State */
                case 13: /* Request Com Port Flow Control Setting (inbound) */
                case 14: /* Use No Flow Control (inbound) */
                case 15: /* Use XON/XOFF Flow Control (inbound) */
                case 16: /* Use HARDWARE Flow Control (inbound) */
                case 17: /* Use DCD Flow Control (outbound/both) */
                case 18: /* Use DTR Flow Control (inbound) */
                case 19: /* Use DSR Flow Control (outbound/both) */
                    ESP_LOGI(TAG, "COM-PORT set-control unsupported %u", value);
                    break;

                case 8: /* Set DTR Signal State ON */
                    ESP_LOGI(TAG, "COM-PORT set-control DTR ON");
                    gpio_set_level(config.dtr_pin, 0);
                    break;

                case 9: /* Set DTR Signal State OFF */
                    ESP_LOGI(TAG, "COM-PORT set-control DTR OFF");
                    gpio_set_level(config.dtr_pin, 1);
                    break;

                case 11: /* Set RTS Signal State ON */
                    ESP_LOGI(TAG, "COM-PORT set-control RTS ON");
                    gpio_set_level(config.rts_pin, 0);
                    break;

                case 12: /* Set RTS Signal State OFF */
                    ESP_LOGI(TAG, "COM-PORT set-control RTS OFF");
                    gpio_set_level(config.rts_pin, 1);
                    break;
            }
            break;

        case TELNET_COM_PORT_PURGE_DATA:
            ESP_LOGI(TAG, "COM-PORT purge-data %u", value);
            uart_flush(config.uart);
            break;

        default:
            ESP_LOGW(TAG, "COM-PORT unknown %u %u", option, value);
            break;
    }
    return 0;
}
#endif

static inline int handle_option(struct telnet_connection_t* conn, unsigned char command, unsigned char option) {
    switch (command) {
        case DO:
        case DONT:
            if (conn->options[option].local_supported) {
                conn->options[option].local_enabled = command == DO ? 1 : 0;
                if (conn->options[option].local_sent) {
                    conn->options[option].local_sent = 0;
                    return 0;
                }
                if (command == DO) {
                    log_command("send", WILL, option);
                    const unsigned char buf[] = {IAC, WILL, option};
                    if (option == TELOPT_TM) {
                        return net_send_all(conn->sock, buf, ARRAYLEN(buf), TAG);
                    }
                    return add_to_outbuf(conn, buf, ARRAYLEN(buf));
                }
            }
            log_command("send", WONT, option);
            return add_to_outbuf(conn, (const unsigned char[]){IAC, WONT, option}, 3);

        case WILL:
        case WONT:
            if (conn->options[option].remote_supported) {
                conn->options[option].remote_enabled = command == WILL ? 1 : 0;
                if (conn->options[option].remote_sent) {
                    conn->options[option].remote_sent = 0;
                    return 0;
                }
                if (command == WILL) {
                    log_command("send", DO, option);
                    if (option == TELOPT_COM_PORT) {
                        conn->outpos = conn->outbuf;
                    }
                    int res = add_to_outbuf(conn, (const unsigned char[]){IAC, DO, option}, 3);
                    if (!res) {
                        switch (option) {
                            case TELOPT_LINEMODE:
#ifdef TELNET_DEBUG
                                ESP_LOGI(TAG, "send IAC SB LINEMODE %u", MODE_EDIT | MODE_TRAPSIG);
#endif
                                return add_to_outbuf(conn, (const unsigned char[]){IAC, SB, TELOPT_LINEMODE, LM_MODE, MODE_EDIT | MODE_TRAPSIG, IAC, SE}, 7);

#ifdef TELNET_UART_SUPPORT
                            case TELOPT_COM_PORT:
                                ESP_ERROR_CHECK(uart_driver_install(config.uart, 2 * UART_FIFO_LEN, 0, 10, &config.uart_queue, 0));
                                xTaskCreate(uart_event_task, "uart_event_task", 2048, conn, 2, &config.uart_event_task);
                                return add_to_outbuf(conn, (const unsigned char[]){IAC, WILL, TELOPT_BINARY}, 3);
#endif
                        }
                    }
                    return res;
                }
            }
            log_command("send", DONT, option);
            return add_to_outbuf(conn, (const unsigned char[]){IAC, DONT, option}, 3);
    }
    return 0;
}

static void handle_client_task(void* data) {
    ESP_LOGI(TAG, "client connected");
    struct net_connection_t* net_conn = data;
    unsigned char buf[BUFFER_SIZE];
    unsigned char prev = 0;
    unsigned char command = 0;
    unsigned char option = 0;
    unsigned char suboption = 0;
    uint32_t value = 0;
    uint8_t val_pos = 0;
    enum {
        READ_REGULAR,
        READ_COMMAND,
        READ_OPTION,
        READ_VALUE,
    } state = READ_REGULAR;

    struct telnet_connection_t conn;
    conn.sock = net_conn->sock;
    conn.inpos = conn.inbuf;
    conn.outpos = conn.outbuf;
    conn.mutex = xSemaphoreCreateMutex();
    memset(conn.options, 0, ARRAYLEN(conn.options));

    conn.options[TELOPT_BINARY].local_supported = 1;
    conn.options[TELOPT_BINARY].remote_supported = 1;
    conn.options[TELOPT_SGA].local_supported = 1;
    conn.options[TELOPT_SGA].remote_supported = 1;
    conn.options[TELOPT_TM].local_supported = 1;
    conn.options[TELOPT_TM].remote_supported = 1;
    conn.options[TELOPT_LINEMODE].remote_supported = 1;
#ifdef TELNET_UART_SUPPORT
    conn.options[TELOPT_COM_PORT].remote_supported = config.uart_enabled;
#endif
    // TODO conn.options[TELOPT_AUTHENTICATION]
    // TODO conn.options[TELOPT_ENCRYPT]
    // TODO conn.options[TELOPT_NEW_ENVIRON]

    if (add_to_outbuf(&conn, prompt, get_prompt_len(&conn))) {
        goto exit_task;
    }

    while (1) {
        ssize_t len = recv(conn.sock, buf, BUFFER_SIZE, 0);
#ifdef TELNET_DEBUG
        ESP_LOGI(TAG, "recv %u bytes", len);
#endif
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: %s", strerror(errno));
            break;
        }
        if (len == 0) {
            ESP_LOGI(TAG, "client disconnected");
            break;
        }
        for (int i = 0; i < len; ++i) {
            unsigned char c = buf[i];
            switch (state) {
                case READ_REGULAR:
                    switch (c) {
                        case '\0':
                            if (conn.options[TELOPT_BINARY].local_enabled) {
                                add_to_line(&conn, c);
                            } else if (prev == '\r') {
                                conn.inpos = conn.inbuf;
                            }
                            break;

                        case '\b':
                            if (conn.options[TELOPT_BINARY].local_enabled) {
                                add_to_line(&conn, c);
                            } else if (conn.inpos > conn.inbuf) {
                                --conn.inpos;
                            }
                            break;

                        case '\n':
                            if (conn.options[TELOPT_BINARY].local_enabled) {
                                add_to_line(&conn, c);
                            } else {
                                if (process_line(&conn)) {
                                    goto exit_task;
                                }
                            }
                            break;

                        case '\r':
                            if (conn.options[TELOPT_BINARY].local_enabled) {
                                add_to_line(&conn, c);
                            }
                            break;

                        case IAC:
                            state = READ_COMMAND;
                            break;

                        default:
                            if (!(c & 0x80) || conn.options[TELOPT_BINARY].local_enabled) {
                                add_to_line(&conn, c);
                            }
                            break;
                    }
                    break;

                case READ_COMMAND:
                    command = c;
                    switch (command) {
                        case IAC:
                            if (conn.options[TELOPT_BINARY].local_enabled) {
                                add_to_line(&conn, c);
                            }
                            state = READ_REGULAR;
                            break;

                        case DO:
                        case DONT:
                        case SB:
                        case WILL:
                        case WONT:
                            state = READ_OPTION;
                            break;

                        default:
                            /* command without option */
                            log_command("recv", command, 0);
                            switch (command) {
                                case AYT: /* are you there */
                                case BREAK:
                                case DM: /* "data mark": we don't really support SYNCH */
                                case GA: /* "go ahead": we don't care as a server */
                                case NOP:
                                    break;

                                case ABORT: /* abort process */
                                case IP:    /* interrupt process--permanently */
                                case SUSP:  /* suspend process */
                                    // TODO cancel subtask
                                    conn.outpos = conn.outbuf;
                                    add_to_outbuf(&conn, (const unsigned char[]){'\r', '\n'}, 2);
                                    add_to_outbuf(&conn, prompt, get_prompt_len(&conn));
                                    break;

                                case AO: /* abort output--but let prog finish */
                                    // TODO cancel subtask output
                                    break;

                                case EC: /* erase the current character */
                                    if (conn.inpos > conn.inbuf) {
                                        --conn.inpos;
                                    }
                                    break;

                                case EL: /* erase the current line */
                                    conn.inpos = conn.inbuf;
                                    break;

                                case EOR: /* end of record (transparent mode) */
                                    if (process_line(&conn)) {
                                        goto exit_task;
                                    }
                                    break;

                                case xEOF: /* end of file */
                                    xSemaphoreTake(conn.mutex, portMAX_DELAY);
                                    send_outbuf(&conn);
                                    xSemaphoreGive(conn.mutex);
                                    ESP_LOGI(TAG, "closing connection");
                                    goto exit_task;

                                default:
                                    ESP_LOGW(TAG, "received unknown command %u", command);
                                    break;
                            }
                            state = READ_REGULAR;
                            break;
                    }
                    break;

                case READ_OPTION:
                    option = c;
                    log_command("recv", command, option);
                    if (command == SB) {
                        val_pos = 0;
                        state = READ_VALUE;
                    } else {
                        /* command with option but without value */
                        if (handle_option(&conn, command, option)) {
                            goto exit_task;
                        }
                        state = READ_REGULAR;
                    }
                    break;

                case READ_VALUE:
                    if (prev == IAC && c != IAC) {
                        if (c == SE) {
#ifdef TELNET_DEBUG
                            ESP_LOGI(TAG, "recv IAC SE");
#endif
#ifdef TELNET_UART_SUPPORT
                            if (option == TELOPT_COM_PORT && suboption == TELNET_COM_PORT_SET_BAUDRATE) {
                                if (handle_com_port_option(&conn, suboption, value)) {
                                    goto exit_task;
                                }
                            }
#endif
                            state = READ_REGULAR;
                        }
                        break;
                    }
#ifdef TELNET_DEBUG
                    ESP_LOGI(TAG, "recv value %u", c);
#endif
                    if (c != IAC || prev == IAC) {
                        switch (option) {
                            case TELOPT_LINEMODE:
                                if (val_pos == 1) {
                                    switch (prev) {
                                        case DO:
                                        case DONT:
                                        case WILL:
                                        case WONT:
                                            if (c == LM_FORWARDMASK) {
#ifdef TELNET_DEBUG
                                                ESP_LOGI(TAG, "send IAC SB LINEMODE %s LM_FORWARDMASK IAC SE", (prev == DO || prev == DONT) ? "WONT" : "DONT");
#endif
                                                if (add_to_outbuf(&conn,
                                                                  (const unsigned char[]){IAC, SB, TELOPT_LINEMODE, (prev == DO || prev == DONT) ? WONT : DONT,
                                                                                          LM_FORWARDMASK, IAC, SE},
                                                                  7)) {
                                                    goto exit_task;
                                                }
                                            }
                                            break;

                                        default:
                                            break;
                                    }
                                }
                                break;

#ifdef TELNET_UART_SUPPORT
                            case TELOPT_COM_PORT:
                                if (val_pos == 0) {
                                    suboption = c;
                                    value = 0;
                                } else {
                                    if (suboption == TELNET_COM_PORT_SET_BAUDRATE) {
                                        value = (value << 8) | c;
                                    } else if (handle_com_port_option(&conn, suboption, c)) {
                                        goto exit_task;
                                    }
                                }
                                break;
#endif
                        }
                    }
                    ++val_pos;
                    break;
            }
            prev = c;
        }
        if (conn.options[TELOPT_BINARY].local_enabled && conn.inpos > conn.inbuf) {
            if (process_line(&conn)) {
                goto exit_task;
            }
        }
        xSemaphoreTake(conn.mutex, portMAX_DELAY);
        if (send_outbuf(&conn)) {
            xSemaphoreGive(conn.mutex);
            goto exit_task;
        }
        xSemaphoreGive(conn.mutex);
    }

exit_task:
    // TODO cancel subtask
#ifdef TELNET_UART_SUPPORT
    if (conn.options[TELOPT_COM_PORT].remote_enabled) {
        uart_driver_delete(config.uart);
        vTaskDelete(config.uart_event_task);
    }
#endif
    net_free_connection(net_conn);
}

static struct net_server_t server = {
    .port = 23,
    .max_connections = 1,
    .client_task = handle_client_task,
    .client_task_stack_depth = 4096,
    .client_task_priority = 2,
};

void telnet_start() {
#ifdef TELNET_UART_SUPPORT
    config.uart_enabled = 0;
#else
    (void)add_to_outbuf_escaped;
#endif
    net_start_server(&server, 1);
}

#ifdef TELNET_UART_SUPPORT
void telnet_start_with_uart(uart_port_t uart, gpio_num_t dtr_pin, gpio_num_t rts_pin) {
    config.uart_enabled = 1;
    config.uart = uart;
    config.dtr_pin = dtr_pin;
    config.rts_pin = rts_pin;
    ESP_ERROR_CHECK(gpio_set_direction(dtr_pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(rts_pin, GPIO_MODE_OUTPUT));
    net_start_server(&server, 1);
}
#endif

void telnet_stop() { net_stop_server(&server); }
