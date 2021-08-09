#include "dns.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <lwip/sockets.h>

#include "net.h"
#include "wifi.h"

#define PORT 53
#define USE_IPV4

#define MAX_LABEL_SIZE 63
#define MAX_NAME_SIZE 255
#define MAX_UDP_MESSAGE_SIZE 512
#define BUFFER_SIZE (MAX_UDP_MESSAGE_SIZE + 1)

struct dns_header_t {
    uint16_t id; /* A 16 bit identifier assigned by the program that generates any kind of query.  This identifier is copied the corresponding reply and can be
                    used by the requester to match up replies to outstanding queries. */
    union {
        struct {
            uint16_t rcode : 4; /* Response code - this 4 bit field is set as part of responses. */
            uint16_t z : 3;     /* Reserved for future use.  Must be zero in all queries and responses. */
            uint16_t ra : 1; /* Recursion Available - this be is set or cleared in a response, and denotes whether recursive query support is available in the
                                name server. */
            uint16_t rd : 1; /* Recursion Desired - this bit may be set in a query and is copied into the response.  If RD is set, it directs the name server to
                                pursue the query recursively. Recursive query support is optional. */
            uint16_t tc : 1; /* TrunCation - specifies that this message was truncated due to length greater than that permitted on the transmission channel. */
            uint16_t aa : 1; /* Authoritative Answer - this bit is valid in responses, and specifies that the responding name server is an authority for the
                                domain name in question section. */
            uint16_t opcode : 4; /* A four bit field that specifies kind of query in this message.  This value is set by the originator of a query and copied
                                    into the response. */
            uint16_t qr : 1;     /* A one bit field that specifies whether this message is a query (0), or a response (1). */
        } __attribute__((packed));
        uint16_t control;
    };
    uint16_t qdcount; /* an unsigned 16 bit integer specifying the number of entries in the question section. */
    uint16_t ancount; /* an unsigned 16 bit integer specifying the number of resource records in the answer section. */
    uint16_t nscount; /* an unsigned 16 bit integer specifying the number of name server resource records in the authority records section. */
    uint16_t arcount; /* an unsigned 16 bit integer specifying the number of resource records in the additional records section. */
} __attribute__((packed));

enum {
    QR_QUERY = 0,
    QR_RESPONSE = 1,
};

enum {
    OPCODE_QUERY = 0,  /* a standard query (QUERY) */
    OPCODE_IQUERY = 1, /* an inverse query (IQUERY) */
    OPCODE_STATUS = 2, /* a server status request (STATUS) */
};

enum {
    RCODE_NO_ERROR = 0,       /* No error condition */
    RCODE_FORMAT_ERROR = 1,   /* Format error - The name server was unable to interpret the query. */
    RCODE_SERVER_FAILURE = 2, /* Server failure - The name server was unable to process this query due to a problem with the name server. */
    RCODE_NAME_ERROR = 3, /* Name Error - Meaningful only for responses from an authoritative name server, this code signifies that the domain name referenced
                             in the query does not exist. */
    RCODE_NOT_IMPLEMENTED = 4, /* Not Implemented - The name server does not support the requested kind of query. */
    RCODE_REFUSED = 5, /* Refused - The name server refuses to perform the specified operation for policy reasons.  For example, a name server may not wish to
                          provide the information to the particular requester, or a name server may not wish to perform a particular operation (e.g., zone
                          transfer) for particular data. */
};

struct dns_question_info_t {
    /* before: qname - a domain name represented as a sequence of labels, where each label consists of a length octet followed by that number of octets.  The
     * domain name terminates with the zero length octet for the null label of the root.  Note that this field may be an odd number of octets; no padding is
     * used. */
    uint16_t qtype; /* a two octet code which specifies the type of the query. The values for this field include all codes valid for a TYPE field, together with
                       some more general codes which can match more than one type of RR. */
    uint16_t qclass; /* a two octet code that specifies the class of the query. For example, the QCLASS field is IN for the Internet. */
} __attribute__((packed));

struct dns_resource_record_info_t {
    /* before: name - a domain name to which this resource record pertains. */
    uint16_t type;     /* two octets containing one of the RR type codes.  This field specifies the meaning of the data in the RDATA field. */
    uint16_t class;    /* two octets which specify the class of the data in the RDATA field. */
    uint32_t ttl;      /* a 32 bit unsigned integer that specifies the time interval (in seconds) that the resource record may be cached before it should be
                          discarded.  Zero values are interpreted to mean that the RR can only be used for the transaction in progress, and should not be cached. */
    uint16_t rdlength; /* an unsigned 16 bit integer that specifies the length in octets of the RDATA field. */
    /* after: rdata -  a variable length string of octets that describes the resource.  The format of this information varies according to the TYPE and CLASS of
     * the resource record. For example, the if the TYPE is A and the CLASS is IN, the RDATA field is a 4 octet ARPA Internet address. */
} __attribute__((packed));

enum {
    TYPE_A = 1,        /* a host address */
    TYPE_NS = 2,       /* an authoritative name server */
    TYPE_MD = 3,       /* a mail destination (Obsolete - use MX) */
    TYPE_MF = 4,       /* a mail forwarder (Obsolete - use MX) */
    TYPE_CNAME = 5,    /* the canonical name for an alias */
    TYPE_SOA = 6,      /* marks the start of a zone of authority */
    TYPE_MB = 7,       /* a mailbox domain name (EXPERIMENTAL) */
    TYPE_MG = 8,       /* a mail group member (EXPERIMENTAL) */
    TYPE_MR = 9,       /* a mail rename domain name (EXPERIMENTAL) */
    TYPE_NULL = 10,    /* a null RR (EXPERIMENTAL) */
    TYPE_WKS = 11,     /* a well known service description */
    TYPE_PTR = 12,     /* a domain name pointer */
    TYPE_HINFO = 13,   /* host information */
    TYPE_MINFO = 14,   /* mailbox or mail list information */
    TYPE_MX = 15,      /* mail exchange */
    TYPE_TXT = 16,     /* text strings */
    QTYPE_AXFR = 252,  /* A request for a transfer of an entire zone */
    QTYPE_MAILB = 253, /* A request for mailbox-related records (MB, MG or MR) */
    QTYPE_MAILA = 254, /* A request for mail agent RRs (Obsolete - see MX) */
    QTYPE_ALL = 255,   /* A request for all records */
};

enum {
    CLASS_IN = 1,     /* the Internet */
    CLASS_CS = 2,     /* the CSNET class (Obsolete - used only for examples in some obsolete RFCs) */
    CLASS_CH = 3,     /* the CHAOS class */
    CLASS_HS = 4,     /* Hesiod [Dyer 87] */
    QCLASS_ANY = 255, /* any class */
};

static TaskHandle_t server_task_handle;

static const char* TAG = "dns";
static int sock = -1;
struct net_loopback_sock_t loopback;

struct dns_question_info_t* read_question(char** buf, ssize_t len) {
    unsigned char* pos = *(unsigned char**)buf;
    while (pos[0] != '\0') {
        unsigned char label_len = pos[0] + 1;
        if (label_len >= len) {
            return NULL;
        }
        len -= label_len;
        pos += label_len;
    }
    *buf = (char*)(pos + 1 + sizeof(struct dns_question_info_t));
    return (struct dns_question_info_t*)(pos + 1);
}

static void server_task() {
    if (net_loopback_sock_new(&loopback)) {
        ESP_LOGE(TAG, "net_loopback_sock_new failed");
        goto exit_task;
    }

#ifdef USE_IPV4
    struct sockaddr_in dest = {
        .sin_addr = {.s_addr = htonl(INADDR_ANY)},
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
    };
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
#else
    struct sockaddr_in6 dest = {
        .sin6_addr = {0},
        .sin6_family = AF_INET6,
        .sin6_port = htons(PORT),
    };
    sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_IPV6);
#endif

    if (sock < 0) {
        ESP_LOGE(TAG, "socket failed: %s", strerror(errno));
        goto exit_task;
    }

    if (net_make_reusable(sock)) {
        ESP_LOGE(TAG, "net_make_reusable failed: %s", strerror(errno));
        goto exit_task;
    }

    if (bind(sock, (struct sockaddr*)&dest, sizeof(dest))) {
        ESP_LOGE(TAG, "bind failed: %s", strerror(errno));
        goto exit_task;
    }

#ifdef USE_IPV4
    struct sockaddr_in source;
#else
    struct sockaddr_in6 source;
#endif
    socklen_t addrlen = sizeof(source);

    char buf[BUFFER_SIZE];

    ESP_LOGI(TAG, "waiting for data on port %u", PORT);
    while (1) {
        if (net_loopback_select(&loopback, sock)) {
            ESP_LOGI(TAG, "received shutdown signal");
            break;
        }
        ssize_t len = recvfrom(sock, buf, BUFFER_SIZE, 0, (struct sockaddr*)&source, &addrlen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: %s", strerror(errno));
            break;
        }
        if (len > MAX_UDP_MESSAGE_SIZE || len < sizeof(struct dns_header_t)) {
            continue;
        }

        struct dns_header_t* header = (struct dns_header_t*)buf;
        header->control = ntohs(header->control);
        char* pos = buf + sizeof(struct dns_header_t);
        ssize_t answer_len = len;

        if (header->qr != QR_QUERY) {
            continue;
        }

        header->ra = 0;
        header->z = 0;

        if (header->opcode != OPCODE_QUERY || ntohs(header->qdcount) != 1 || header->tc) {
            header->rcode = RCODE_NOT_IMPLEMENTED;
            goto send_reply;
        }

        if (header->ancount != 0) {
            header->rcode = RCODE_FORMAT_ERROR;
            goto send_reply;
        }

        struct dns_question_info_t* question_info = read_question(&pos, len);
        if (!question_info) {
            header->rcode = RCODE_FORMAT_ERROR;
            goto send_reply;
        }

        switch (ntohs(question_info->qtype)) {
            case TYPE_A:
            case QTYPE_ALL:
                break;

            default:
                header->rcode = RCODE_NOT_IMPLEMENTED;
                goto send_reply;
        }

        switch (ntohs(question_info->qclass)) {
            case CLASS_IN:
            case QCLASS_ANY:
                break;

            default:
                header->rcode = RCODE_NOT_IMPLEMENTED;
                goto send_reply;
        }

        answer_len = (pos - buf) + sizeof(uint16_t) + sizeof(struct dns_resource_record_info_t) + sizeof(uint32_t);
        if (answer_len > MAX_UDP_MESSAGE_SIZE) {
            answer_len = len;
            header->rcode = RCODE_FORMAT_ERROR;
            goto send_reply;
        }

        header->qdcount = htons(1);
        header->ancount = htons(1);

        uint16_t i16 = 0xc000 | (uint16_t)sizeof(struct dns_header_t); /* use pointer to query label as label */
        /* set byte by byte as alignment might be off */
        pos[0] = (i16 >> 8) & 0xff;
        pos[1] = (i16 >> 0) & 0xff;
        pos += sizeof(uint16_t);

        struct dns_resource_record_info_t* resource_record_info = (struct dns_resource_record_info_t*)pos;
        resource_record_info->type = htons(TYPE_A);
        resource_record_info->class = htons(CLASS_IN);
        resource_record_info->ttl = htonl(60);
        resource_record_info->rdlength = htons((uint16_t)sizeof(uint32_t));
        pos += sizeof(struct dns_resource_record_info_t);

        uint32_t i32 = htonl(wifi_get_local_ip());
        /* set byte by byte as alignment might be off */
        pos[0] = (i32 >> 24) & 0xff;
        pos[1] = (i32 >> 16) & 0xff;
        pos[2] = (i32 >> 8) & 0xff;
        pos[3] = (i32 >> 0) & 0xff;
        pos += sizeof(uint32_t);

    send_reply:
        header->qr = QR_RESPONSE;
        header->nscount = 0;
        header->arcount = 0;

        header->control = htons(header->control);
        if (sendto(sock, buf, answer_len, 0, (struct sockaddr*)&source, addrlen) < 0) {
            ESP_LOGE(TAG, "sendto failed: %s", strerror(errno));
        }
    }

exit_task:
    if (sock >= 0) {
        shutdown(sock, SHUT_RDWR);
        close(sock);
        sock = -1;
        ESP_LOGI(TAG, "stopped server on port %u", PORT);
    }
    net_loopback_sock_close(&loopback);
    server_task_handle = NULL;
    vTaskDelete(NULL);
}

void dns_start() { xTaskCreate(server_task, "server_task_handle", 2048, NULL, 1, &server_task_handle); }

void dns_stop() {
    if (sock >= 0) {
        if (net_loopback_sock_send_byte(&loopback, NET_CONTROL_SHUTDOWN) < 0) {
            ESP_LOGE(TAG, "net_loopback_sock_send_byte failed for port %u: %s", loopback.port, strerror(errno));
        }
    }
}
