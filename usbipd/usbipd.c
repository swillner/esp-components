// https://www.kernel.org/doc/html/latest/driver-api/usb/URB.html
// https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html

#include "usbipd.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <lwip/sockets.h>
#include <string.h>

#include "net.h"

#define USBIP_VERSION 0x0111

#define OP_REQUEST 0x8000
#define OP_REPLY 0x0000
#define OP_DEVLIST 0x0005
#define OP_IMPORT 0x0003

#define USBIP_CMD_SUBMIT 0x00000001
#define USBIP_CMD_UNLINK 0x00000002
#define USBIP_RET_SUBMIT 0x00000003
#define USBIP_RET_UNLINK 0x00000004

#define USBIP_DIR_OUT 0
#define USBIP_DIR_IN 1

// after TODO
struct usb_device_descriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} __attribute__((packed));

// after TODO
struct usbipd_header {
    uint16_t version;
    uint16_t code;
    uint32_t status;
} __attribute__((packed));

// from TODO
struct usbip_usb_interface {
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t padding;
} __attribute__((packed));

// from TODO
enum usb_device_speed {
    USB_SPEED_UNKNOWN = 0, /* enumerating */
    USB_SPEED_LOW,
    USB_SPEED_FULL,       /* usb 1.1 */
    USB_SPEED_HIGH,       /* usb 2.0 */
    USB_SPEED_WIRELESS,   /* wireless (usb 2.5) */
    USB_SPEED_SUPER,      /* usb 3.0 */
    USB_SPEED_SUPER_PLUS, /* usb 3.1 */
};

// from TODO
struct usbip_usb_device {
    char path[256];
    char busid[32];

    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;

    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;

    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bConfigurationValue;
    uint8_t bNumConfigurations;
    uint8_t bNumInterfaces;
} __attribute__((packed));

// after drivers/usb/usbip/usbip_common.h
struct usbip_header {
    struct {
        uint32_t command;
        uint32_t seqnum;
        uint32_t devid;
        uint32_t direction;
        uint32_t ep;
    } __attribute__((packed)) base;
    union {
        struct {
            uint32_t transfer_flags;
            uint32_t transfer_buffer_length;
            uint32_t start_frame;
            uint32_t number_of_packets;
            uint32_t interval;
            unsigned char setup[8];
        } __attribute__((packed)) cmd_submit;
        struct {
            int32_t status;
            int32_t actual_length;
            int32_t start_frame;
            int32_t number_of_packets;
            int32_t error_count;
            char padding[8];
        } __attribute__((packed)) ret_submit;
        struct {
            uint32_t seqnum;
            char padding[24];
        } __attribute__((packed)) cmd_unlink;
        struct {
            int32_t status;
            char padding[24];
        } __attribute__((packed)) ret_unlink;
    };
} __attribute__((packed));

// after drivers/usb/usbip/usbip_common.h
struct usbip_iso_packet_descriptor {
    uint32_t offset;
    uint32_t length;
    uint32_t actual_length;
    uint32_t status;
} __attribute__((packed));

// from include/linux/usb.h
#define URB_SHORT_NOT_OK 0x0001        /* report short reads as errors */
#define URB_ISO_ASAP 0x0002            /* iso-only; use the first unexpired slot in the schedule */
#define URB_NO_TRANSFER_DMA_MAP 0x0004 /* urb->transfer_dma valid on submit */
#define URB_ZERO_PACKET 0x0040         /* Finish bulk OUT with short packet */
#define URB_NO_INTERRUPT 0x0080        /* HINT: no non-error interrupt needed */
#define URB_FREE_BUFFER 0x0100         /* Free transfer buffer with the URB */
#define URB_DIR_IN 0x0200              /* Transfer from device to host */
#define URB_DIR_OUT 0
#define URB_DIR_MASK URB_DIR_IN
#define URB_DMA_MAP_SINGLE 0x00010000      /* Non-scatter-gather mapping */
#define URB_DMA_MAP_PAGE 0x00020000        /* HCD-unsupported S-G */
#define URB_DMA_MAP_SG 0x00040000          /* HCD-supported S-G */
#define URB_MAP_LOCAL 0x00080000           /* HCD-local-memory mapping */
#define URB_SETUP_MAP_SINGLE 0x00100000    /* Setup packet DMA mapped */
#define URB_SETUP_MAP_LOCAL 0x00200000     /* HCD-local setup packet */
#define URB_DMA_SG_COMBINED 0x00400000     /* S-G entries were combined */
#define URB_ALIGNED_TEMP_BUFFER 0x00800000 /* Temp buffer was alloc'd */

static const char* TAG = "usbipd";

static inline int handle_devlist_request(struct net_connection_t* conn) {
    /* struct usbip_usb_interface iface = { */
    /*     .bInterfaceClass = 0xe0, */
    /*     .bInterfaceSubClass = 1, */
    /*     .bInterfaceProtocol = 1, */
    /*     .padding = 0, */
    /* }; */
    struct usbip_usb_device dev = {
        .path = "/sys/devices/pci0000:00/0000:00:14.0/usb1/1-3",
        .busid = "1-3",
        .busnum = htonl(1),
        .devnum = htonl(3),
        .speed = htonl(USB_SPEED_FULL),
        .idVendor = htons(0x0458),
        .idProduct = htons(0x706e),
        .bcdDevice = htons(2),
        .bDeviceClass = 0xef,
        .bDeviceSubClass = 0x02,
        .bDeviceProtocol = 0x01,
        .bConfigurationValue = 1,
        .bNumConfigurations = 1,
        .bNumInterfaces = 0,
    };

    ESP_LOGI(TAG, "received devlist request");
    struct {
        struct usbipd_header header;
        uint32_t ndev;
    } __attribute__((packed)) reply = {
        .header =
            {
                .version = htons(USBIP_VERSION),
                .code = htons(OP_DEVLIST | OP_REPLY),
                .status = 0,
            },
        .ndev = htonl(1),
    };
    if (net_send_all(conn->sock, &reply, sizeof(reply), TAG) < 0) {
        return 1;
    }
    if (net_send_all(conn->sock, &dev, sizeof(dev), TAG) < 0) {
        return 1;
    }
    return 0;
}

static inline int handle_exported_device(struct net_connection_t* conn) {
    // TODO check devid
    struct usbip_header header;
    int disconnected;
    while (1) {
        if (net_recv_all(conn->sock, &header, sizeof(header), TAG, &disconnected) < 0 || disconnected) {
            return 1;
        }

        uint32_t seqnum = header.base.seqnum;

        header.base.command = ntohl(header.base.command);
        header.base.seqnum = ntohl(header.base.seqnum);
        header.base.devid = ntohl(header.base.devid);
        header.base.direction = ntohl(header.base.direction);
        header.base.ep = ntohl(header.base.ep);
        ESP_LOGI(TAG, "header.base: command=%u seqnum=%u busnum=%u devnum=%u direction=%s ep=%u", header.base.command, header.base.seqnum,
                 header.base.devid >> 16, header.base.devid & 0xffff, header.base.direction == USBIP_DIR_IN ? "IN" : "OUT", header.base.ep);

        switch (header.base.command) {
            case USBIP_CMD_SUBMIT:
                header.cmd_submit.transfer_flags = ntohl(header.cmd_submit.transfer_flags);
                header.cmd_submit.transfer_buffer_length = ntohl(header.cmd_submit.transfer_buffer_length);
                header.cmd_submit.start_frame = ntohl(header.cmd_submit.start_frame);
                header.cmd_submit.number_of_packets = ntohl(header.cmd_submit.number_of_packets);
                header.cmd_submit.interval = ntohl(header.cmd_submit.interval);

                ESP_LOGI(TAG,
                         "header.cmd_submit: transfer_flags=%#0x transfer_buffer_length=%u start_frame=%u number_of_packets=%u interval=%u "
                         "setup=0x%02x%02x%02x%02x%02x%02x%02x%02x",
                         header.cmd_submit.transfer_flags, header.cmd_submit.transfer_buffer_length, header.cmd_submit.start_frame,
                         header.cmd_submit.number_of_packets, header.cmd_submit.interval, header.cmd_submit.setup[0], header.cmd_submit.setup[1],
                         header.cmd_submit.setup[2], header.cmd_submit.setup[3], header.cmd_submit.setup[4], header.cmd_submit.setup[5],
                         header.cmd_submit.setup[6], header.cmd_submit.setup[7]);

                if (header.base.direction == USBIP_DIR_OUT) {
                    // TODO read header.cmd_submit.transfer_buffer_length bytes
                }
                if (header.cmd_submit.start_frame != 0) {
                    // TODO ISO transfer->read header.cmd_submit.number_of_packets times iso_packet_descriptor
                }

                // create reply
                memset(&header, 0, sizeof(header));
                header.base.command = htonl(USBIP_RET_SUBMIT);
                header.base.seqnum = seqnum;
                /* if (net_send_all(conn->sock, &header, sizeof(header), TAG) < 0) { */
                /*     return 1; */
                /* } */
                break;

            case USBIP_CMD_UNLINK:
                ESP_LOGI(TAG, "header.cmd_unlink: seqnum=%u", ntohl(header.cmd_unlink.seqnum));

                // create reply
                memset(&header, 0, sizeof(header));
                header.base.command = htonl(USBIP_RET_UNLINK);
                header.base.seqnum = seqnum;
                header.ret_unlink.status = htonl(-ECONNRESET);
                if (net_send_all(conn->sock, &header, sizeof(header), TAG) < 0) {
                    return 1;
                }
                break;

            default:
                ESP_LOGE(TAG, "received unsupported command");
                return 1;
        }
    }
    return 0;
}

static inline int handle_import_request(struct net_connection_t* conn) {
    struct usbip_usb_device dev = {
        .path = "/sys/devices/pci0000:00/0000:00:14.0/usb1/1-3",
        .busid = "1-3",
        .busnum = htonl(1),
        .devnum = htonl(3),
        .speed = htonl(USB_SPEED_FULL),
        .idVendor = htons(0x0458),
        .idProduct = htons(0x706e),
        .bcdDevice = htons(2),
        .bDeviceClass = 0xef,
        .bDeviceSubClass = 0x02,
        .bDeviceProtocol = 0x01,
        .bConfigurationValue = 1,
        .bNumConfigurations = 1,
        .bNumInterfaces = 0,
    };

    ESP_LOGI(TAG, "received import request");
    char busid[32];
    int disconnected;
    if (net_recv_all(conn->sock, busid, 32, TAG, &disconnected) < 0 || disconnected) {
        return 1;
    }
    busid[31] = '\0';
    ESP_LOGI(TAG, "import request is for %s", busid);
    struct usbipd_header reply = {
        .version = htons(USBIP_VERSION),
        .code = htons(OP_IMPORT | OP_REPLY),
        .status = 0,
    };
    if (net_send_all(conn->sock, &reply, sizeof(reply), TAG) < 0) {
        return 1;
    }
    if (net_send_all(conn->sock, &dev, sizeof(dev), TAG) < 0) {
        return 1;
    }
    return handle_exported_device(conn);
}

static void handle_client_task(void* data) {
    ESP_LOGI(TAG, "client connected");
    struct net_connection_t* conn = data;

    struct usbipd_header header;

    int disconnected;
    if (net_recv_all(conn->sock, &header, sizeof(header), TAG, &disconnected) || disconnected) {
        goto exit_task;
    }

    if (ntohs(header.version) != USBIP_VERSION) {
        ESP_LOGE(TAG, "unsupported usbip version");
        goto exit_task;
    }

    switch (ntohs(header.code)) {
        case OP_DEVLIST | OP_REQUEST:
            handle_devlist_request(conn);
            break;
        case OP_IMPORT | OP_REQUEST:
            handle_import_request(conn);
            break;
        default:
            ESP_LOGE(TAG, "received unknown opcode");
            break;
    }

exit_task:
    net_free_connection(conn);
}

static struct net_server_t server = {
    .port = 3240,
    .max_connections = 1,
    .client_task = handle_client_task,
    .client_task_stack_depth = 2048,
    .client_task_priority = 2,
};

void usbipd_start() { net_start_server(&server, 1); }
void usbipd_stop() { net_stop_server(&server); }
