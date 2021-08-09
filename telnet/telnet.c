#include "telnet.h"

#include <freertos/FreeRTOS.h>
//
#include <esp_log.h>
#include <lwip/sockets.h>
#include <string.h>

#include "net.h"

#define TELNET_DEBUG
#ifdef TELNET_DEBUG
#define TELCMDS
#define TELOPTS
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#pragma GCC diagnostic ignored "-Wtype-limits"
#include "arpa/telnet.h"
#pragma GCC diagnostic pop

#define TELOPT_COM_PORT 44  // TODO

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
    struct telnet_option_t options[TELOPT_LAST + 1];
    unsigned char inbuf[MAX_LINE_LENGTH];
    unsigned char* inpos;
    unsigned char outbuf[MAX_LINE_LENGTH];
    unsigned char* outpos;
};

static inline int get_prompt_len(struct telnet_connection_t* conn) { return ARRAYLEN(prompt) - (conn->options[TELOPT_SGA].local_enabled ? 2 : 0); }

static int send_outbuf(struct telnet_connection_t* conn) {
    const int len = conn->outpos - conn->outbuf;
#ifdef TELNET_DEBUG
    ESP_LOGI(TAG, "send %u bytes", len);
#endif
    if (net_send_all(conn->sock, conn->outbuf, len, TAG) < 0) {
        return 1;
    }
    conn->outpos = conn->outbuf;
    return 0;
}

static int add_to_outbuf(struct telnet_connection_t* conn, const unsigned char* buf, int len) {
    int left = conn->outbuf + ARRAYLEN(conn->outbuf) - conn->outpos;
    while (1) {
        int l = left < len ? left : len;
        memcpy(conn->outpos, buf, l);
        conn->outpos += l;
        len -= l;
        if (len == 0) {
            return 0;
        }
        if (send_outbuf(conn)) {
            return 1;
        }
        left = ARRAYLEN(conn->outbuf);
    }
}

static int process_line(struct telnet_connection_t* conn) {
    ESP_LOGI(TAG, "received line %.*s", conn->inpos - conn->inbuf, conn->inbuf);
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

static inline int handle_option(struct telnet_connection_t* conn, unsigned char command, unsigned char option) {
    if (!TELOPT_OK(option)) {
        return 0;
    }
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
                    int res = add_to_outbuf(conn, (const unsigned char[]){IAC, DO, option}, 3);
                    if (option == TELOPT_LINEMODE && !res) {
#ifdef TELNET_DEBUG
                        ESP_LOGI(TAG, "send IAC SB LINEMODE %u", MODE_EDIT | MODE_TRAPSIG);
#endif
                        return add_to_outbuf(conn, (const unsigned char[]){IAC, SB, TELOPT_LINEMODE, LM_MODE, MODE_EDIT | MODE_TRAPSIG, IAC, SE}, 7);
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
    unsigned char val_prev = 0;
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
    memset(conn.options, 0, ARRAYLEN(conn.options));

    conn.options[TELOPT_BINARY].local_supported = 1;
    conn.options[TELOPT_BINARY].remote_supported = 1;
    conn.options[TELOPT_SGA].local_supported = 1;
    conn.options[TELOPT_SGA].remote_supported = 1;
    conn.options[TELOPT_TM].local_supported = 1;
    conn.options[TELOPT_TM].remote_supported = 1;
    conn.options[TELOPT_LINEMODE].remote_supported = 1;
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
                                    send_outbuf(&conn);
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
                        val_prev = 0;
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
                            state = READ_REGULAR;
                        }
                        break;
                    }
#ifdef TELNET_DEBUG
                    ESP_LOGI(TAG, "recv value %u", c);
#endif
                    switch (option) {
                        case TELOPT_LINEMODE:
                            if (val_pos == 1) {
                                switch (val_prev) {
                                    case DO:
                                    case DONT:
                                    case WILL:
                                    case WONT:
                                        if (c == LM_FORWARDMASK) {
#ifdef TELNET_DEBUG
                                            ESP_LOGI(TAG, "send IAC SB LINEMODE %s LM_FORWARDMASK IAC SE",
                                                     (val_prev == DO || val_prev == DONT) ? "WONT" : "DONT");
#endif
                                            if (add_to_outbuf(
                                                    &conn,
                                                    (const unsigned char[]){IAC, SB, TELOPT_LINEMODE, (val_prev == DO || val_prev == DONT) ? WONT : DONT,
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
                    }
                    val_prev = IAC;
                    ++val_pos;
                    if (prev == IAC && c == IAC) {
                        c = 0; /* make sure prev != IAC next loop iteration */
                    }
                    break;
            }
            prev = c;
        }
        if (send_outbuf(&conn)) {
            goto exit_task;
        }
    }

exit_task:
    // TODO cancel subtask
    net_free_connection(net_conn);
}

static struct net_server_t server = {
    .port = 23,
    .max_connections = 1,
    .client_task = handle_client_task,
    .client_task_stack_depth = 4096,
    .client_task_priority = 2,
};

void telnet_start() { net_start_server(&server, 1); }
void telnet_stop() { net_stop_server(&server); }
