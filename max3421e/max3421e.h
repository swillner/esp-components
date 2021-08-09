#ifndef MAX3421E_H
#define MAX3421E_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
//
#include <driver/gpio.h>

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
} __attribute__((packed));

struct usb_setup_t {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed));

struct max3421e_handler_data_t {
    spi_t spi;
    TaskHandle_t check_irqs_task_handle;
    EventGroupHandle_t irq_events;
    SemaphoreHandle_t spi_mutex;
};

struct max3421e_transfer_t {
    struct usb_setup_t* setup;
    uint32_t* data;
    uint8_t addr;
    uint8_t endpoint;
};

int max3421e_control_transfer(struct max3421e_handler_data_t* data, uint8_t addr, uint8_t endpoint, struct usb_setup_t* setup, void* buf);
int max3421e_get_device_descriptor(struct max3421e_handler_data_t* data, struct usb_device_descriptor_t* buf);
int max3421e_wait_for_connection(struct max3421e_handler_data_t* data);
int max3421e_init(spi_t spi, gpio_num_t int_gpio, gpio_num_t reset_gpio, struct max3421e_handler_data_t* data);

#endif
