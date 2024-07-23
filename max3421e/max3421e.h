#ifndef MAX3421E_H
#define MAX3421E_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
//
#include <driver/gpio.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)

#define USE_SPI_DEVICE
#include <driver/spi_master.h>
typedef spi_device_handle_t spi_t;

#else

#include <driver/spi.h>
typedef spi_host_t spi_t;

#endif

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
} __attribute__((aligned(4),packed));
#define USB_SIZE_DEVICE_DESCRIPTOR 18

struct usb_config_descriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __attribute__((aligned(4),packed));
#define USB_SIZE_CONFIG_DESCRIPTOR 9

struct usb_interface_descriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __attribute__((packed));
#define USB_SIZE_INTERFACE_DESCRIPTOR 9

struct usb_endpoint_descriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} __attribute__((packed));
#define USB_SIZE_ENDPOINT_DESCRIPTOR 7

struct usb_setup_t {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((aligned(4),packed));
#define USB_SIZE_SETUP 8

struct max3421e_endpoint_t {
    uint8_t id;
    uint8_t packetsize;
    uint8_t datatoggles;
};

struct max3421e_handle_t {
    spi_t spi;
    TaskHandle_t check_irqs_task_handle;
    EventGroupHandle_t irq_events;
    SemaphoreHandle_t spi_mutex;
    uint8_t highspeed;
    uint8_t last_endpoint_used;
    struct max3421e_endpoint_t ep0;
};

int max3421e_bulk_out(struct max3421e_handle_t* handle, struct max3421e_endpoint_t* endpoint, uint8_t* data, size_t len);
int max3421e_control_transfer(struct max3421e_handle_t* handle, struct usb_setup_t* setup, void* buf);
int max3421e_get_device_descriptor(struct max3421e_handle_t* handle, struct usb_device_descriptor_t* desc);
int max3421e_init(spi_t spi, gpio_num_t int_gpio, gpio_num_t reset_gpio, struct max3421e_handle_t* handle);
int max3421e_select_configuration(struct max3421e_handle_t* handle, uint16_t configuration);
int max3421e_setup(struct max3421e_handle_t* handle);
int max3421e_wait_for_connection(struct max3421e_handle_t* handle, TickType_t timeout);
ssize_t max3421e_bulk_in(struct max3421e_handle_t* handle, struct max3421e_endpoint_t* endpoint, uint8_t* data, size_t len);
void max3421e_select_addr(struct max3421e_handle_t* handle, uint8_t addr);
int max3421e_get_config_descriptor(struct max3421e_handle_t* handle, uint16_t index, struct usb_config_descriptor_t* desc);
int max3421e_get_full_configuration(struct max3421e_handle_t* handle, uint16_t index, uint8_t* data, size_t len);
int max3421e_get_status(struct max3421e_handle_t* handle);
int max3421e_is_configured(struct max3421e_handle_t* handle);

void usb_log_device_descriptor(const char* tag, struct usb_device_descriptor_t* desc);
void usb_log_config_descriptor(const char* tag, struct usb_config_descriptor_t* desc);
void usb_log_interface_descriptor(const char* tag, struct usb_interface_descriptor_t* desc);
void usb_log_endpoint_descriptor(const char* tag, struct usb_endpoint_descriptor_t* desc);

#endif
