#include "max3421e.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <string.h>

#include "max3421e_internal.h"

// TODO what about failing spi?

#define PREFIX_E "\033[0;31mE"
#define PREFIX_I "\033[0;32mI"
#define PREFIX_W "\033[0;33mW"
#define PREFIX_D "D"
#define PREFIX_BLUE "\033[0;34m "
#define PREFIX_MAGENTA "\033[0;35m "
#define PREFIX_LIGHTBLUE "\033[0;36m "
#define ESP_LOG_START(tag, prefix) ets_printf("%s (%d) %s: ", prefix, esp_log_early_timestamp(), tag)
#define ESP_LOG_WRITE(format, ...) ets_printf(format, ##__VA_ARGS__)
#define ESP_LOG_END() ets_printf("\033[0m\n")

#define DEFAULT_TIMEOUT pdMS_TO_TICKS(3000)

static const char* TAG = "max3421e";

static inline void spi_transfer(struct max3421e_handle_t* handle,
#ifdef USE_SPI_DEVICE
                                spi_transaction_t* trans
#else
                                spi_trans_t* trans
#endif
) {
    if (xSemaphoreTake(handle->spi_mutex, DEFAULT_TIMEOUT) == pdFAIL) {
        ESP_LOGE(TAG, "spi mutex timeout");
        abort();
    }
#ifdef USE_SPI_DEVICE
    spi_device_transmit(handle->spi, trans);
#else
    spi_trans(handle->spi, trans);
#endif
    xSemaphoreGive(handle->spi_mutex);
}

static uint8_t IRAM_ATTR read_register(struct max3421e_handle_t* handle, uint8_t reg) {
    uint16_t cmd = (reg << 3) | CMD_READ;
#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .flags = SPI_TRANS_USE_RXDATA,
        .rxlength = 8,
    };
    spi_transfer(handle, &trans);
    return trans.rx_data[0];
#else
    uint32_t res = 0;
    spi_trans_t trans = {.cmd = &cmd,
                         .addr = NULL,
                         .mosi = NULL,
                         .miso = &res,
                         .bits = {
                             .cmd = 8,
                             .addr = 0,
                             .mosi = 0,
                             .miso = 8,
                         }};
    spi_transfer(handle, &trans);
    return res;
#endif
}

static void IRAM_ATTR write_register(struct max3421e_handle_t* handle, uint8_t reg, uint8_t val) {
    uint16_t cmd = (reg << 3) | CMD_WRITE;
#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_data = {val, 0, 0, 0},
    };
    spi_transfer(handle, &trans);
#else
    uint32_t val32 = val;
    spi_trans_t trans = {.cmd = &cmd,
                         .addr = NULL,
                         .mosi = &val32,
                         .miso = NULL,
                         .bits = {
                             .cmd = 8,
                             .addr = 0,
                             .mosi = 8,
                             .miso = 0,
                         }};
    spi_transfer(handle, &trans);
#endif
}

static const char* get_result_description(enum max3421e_result_t code) {
    switch (code) {
        case hrSUCCESS:
            return "hrSUCCESS: Successful Transfer";
        case hrBUSY:
            return "hrBUSY: SIE is busy, transfer pending";
        case hrBADREQ:
            return "hrBADREQ: Bad value in HXFR reg";
        case hrUNDEF:
            return "hrUNDEF: (reserved)";
        case hrNAK:
            return "hrNAK: Peripheral returned NAK";
        case hrSTALL:
            return "hrSTALL: Perpheral returned STALL";
        case hrTOGERR:
            return "hrTOGERR: Toggle error/ISO over-underrun";
        case hrWRONGPID:
            return "hrWRONGPID: Received the wrong PID";
        case hrBADBC:
            return "hrBADBC: Bad byte count";
        case hrPIDERR:
            return "hrPIDERR: Receive PID is corrupted";
        case hrPKTERR:
            return "hrPKTERR: Packet error (stuff, EOP)";
        case hrCRCERR:
            return "hrCRCERR: CRC error";
        case hrKERR:
            return "hrKERR: K-state instead of response";
        case hrJERR:
            return "hrJERR: J-state instead of response";
        case hrTIMEOUT:
            return "hrTIMEOUT: Device did not respond in time";
        case hrBABBLE:
            return "hrBABBLE: Device talked too long";
        default:
            return "unknown";
    }
}

static enum max3421e_result_t launch_transfer(struct max3421e_handle_t* handle, uint8_t type, struct max3421e_endpoint_t* endpoint) {
    ESP_LOG_START(TAG, PREFIX_BLUE);
    switch (type) {
        case VAL_HXFR_SETUP:
            ESP_LOG_WRITE("SETUP");
            break;
        case VAL_HXFR_BULK_IN:
            ESP_LOG_WRITE("BULK in");
            break;
        case VAL_HXFR_BULK_OUT:
            ESP_LOG_WRITE("BULK out");
            break;
        case VAL_HXFR_HS_IN:
            ESP_LOG_WRITE("HS in");
            break;
        case VAL_HXFR_HS_OUT:
            ESP_LOG_WRITE("HS out");
            break;
        case VAL_HXFR_ISO_IN:
            ESP_LOG_WRITE("ISO in");
            break;
        case VAL_HXFR_ISO_OUT:
            ESP_LOG_WRITE("ISO out");
            break;
    }
    ESP_LOG_WRITE(" ep %u...", endpoint->id);
    xEventGroupClearBits(handle->irq_events, BIT_HIRQ_HXFRDNIRQ);
    for (int i = 0; i < 0xff; ++i) {
        write_register(handle, REG_HXFR, type | endpoint->id);
        if (!(xEventGroupWaitBits(handle->irq_events, BIT_HIRQ_HXFRDNIRQ, pdTRUE, pdFALSE, DEFAULT_TIMEOUT) & BIT_HIRQ_HXFRDNIRQ)) {
            ESP_LOG_WRITE("\033[0;31mtimeout");
            ESP_LOG_END();
            return hrTIMEOUT;
        }
        /* if (type == VAL_HXFR_BULK_IN) { */
        /*     xEventGroupWaitBits(handle->irq_events, BIT_HIRQ_RCVDAVIRQ, pdFALSE, pdFALSE, DEFAULT_TIMEOUT);  // TODO */
        /* } */
        uint8_t hrsl = read_register(handle, REG_HRSL);
        enum max3421e_result_t res = hrsl & MASK_HRSL_HRSLT3;
        switch (res) {
            case hrSUCCESS:
                ESP_LOG_WRITE("done");
                ESP_LOG_END();
                endpoint->datatoggles = hrsl;
                return hrSUCCESS;

            case hrNAK:
                /* ESP_LOG_WRITE("nak"); */
                /* ESP_LOG_END(); */
                /* endpoint->datatoggles = hrsl; */
                /* return hrNAK;  // TODO */
                if (i % 10 == 0) {
                    ESP_LOG_WRITE(".");
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case hrTOGERR:
                ESP_LOG_WRITE("*");
                switch (type) {
                    case VAL_HXFR_BULK_IN:
                    case VAL_HXFR_HS_IN:
                    case VAL_HXFR_ISO_IN:
                        write_register(handle, REG_HCTL, (hrsl & BIT_HRSL_RCVTOGRD) ? BIT_HCTL_RCVTOG1 : BIT_HCTL_RCVTOG0);
                        break;
                    case VAL_HXFR_SETUP:
                    case VAL_HXFR_BULK_OUT:
                    case VAL_HXFR_HS_OUT:
                    case VAL_HXFR_ISO_OUT:
                        write_register(handle, REG_HCTL, (hrsl & BIT_HRSL_SNDTOGRD) ? BIT_HCTL_SNDTOG1 : BIT_HCTL_SNDTOG0);
                        break;
                }
                break;

            default:
                ESP_LOG_WRITE("\033[0;31mfailed with %s", get_result_description(res));
                ESP_LOG_END();
                return res;
        }
    }
    ESP_LOG_WRITE("nak");
    ESP_LOG_END();
    return hrNAK;
}

static int in_transfer(struct max3421e_handle_t* handle, uint8_t type, struct max3421e_endpoint_t* endpoint, void* data, uint8_t len, uint8_t* total_received) {
    *total_received = 0;

    enum max3421e_result_t st = launch_transfer(handle, type, endpoint);
    if (st == hrNAK) {
        return 0;
    }
    if (st != hrSUCCESS) {
        return -1;
    }

    if (!(xEventGroupGetBits(handle->irq_events) & BIT_HIRQ_RCVDAVIRQ)) {
        ESP_LOGE(TAG, "RCVDAVIRQ should have been set (0x%02x)", xEventGroupGetBits(handle->irq_events));
        return -1;
    }

    do {
        uint8_t received = read_register(handle, REG_RCVBC);
        ESP_LOGI(TAG, "received %u bytes", received);
        if (received > len) {
            ESP_LOGE(TAG, "buffer overflow");
            return -1;
        }
        uint16_t cmd = (REG_RCVFIFO << 3) | CMD_READ;

#ifdef USE_SPI_DEVICE
        spi_transaction_t trans = {
            .cmd = cmd,
            .rxlength = 8 * received,
            .rx_buffer = data,
        };
        spi_transfer(handle, &trans);
#else
        spi_trans_t trans = {.cmd = &cmd,
                             .addr = NULL,
                             .mosi = NULL,
                             .miso = data,
                             .bits = {
                                 .cmd = 8,
                                 .addr = 0,
                                 .mosi = 0,
                                 .miso = 8 * received,
                             }};
        spi_transfer(handle, &trans);
#endif
        xEventGroupClearBits(handle->irq_events, BIT_HIRQ_RCVDAVIRQ);
        write_register(handle, REG_HIRQ, BIT_HIRQ_RCVDAVIRQ);
        data += received;
        len -= received;
        *total_received += received;
    } while (read_register(handle, REG_HIRQ) & BIT_HIRQ_RCVDAVIRQ);

    return 0;
}

static int out_transfer(struct max3421e_handle_t* handle, uint8_t type, struct max3421e_endpoint_t* endpoint, uint8_t fifo_reg, void* data, uint8_t len) {
    uint16_t cmd = (fifo_reg << 3) | CMD_WRITE;
#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .length = 8 * len,
        .tx_buffer = data,
    };
    spi_transfer(handle, &trans);
#else
    spi_trans_t trans = {.cmd = &cmd,
                         .addr = NULL,
                         .mosi = data,
                         .miso = NULL,
                         .bits = {
                             .cmd = 8,
                             .addr = 0,
                             .mosi = 8 * len,
                             .miso = 0,
                         }};
    spi_transfer(handle, &trans);
#endif
    if (fifo_reg == REG_SNDFIFO) {
        write_register(handle, REG_SNDBC, len);
    }
    return launch_transfer(handle, type, endpoint) != hrSUCCESS;
}

static void max3421e_choose_endpoint(struct max3421e_handle_t* handle, int dir_read, const struct max3421e_endpoint_t* endpoint) {
    if (handle->last_endpoint_used != endpoint->id) {
        handle->last_endpoint_used = endpoint->id;
        if (dir_read) {
            write_register(handle, REG_HCTL, (endpoint->datatoggles & BIT_HRSL_RCVTOGRD) ? BIT_HCTL_RCVTOG1 : BIT_HCTL_RCVTOG0);
        } else {
            write_register(handle, REG_HCTL, (endpoint->datatoggles & BIT_HRSL_SNDTOGRD) ? BIT_HCTL_SNDTOG1 : BIT_HCTL_SNDTOG0);
        }
    }
}

ssize_t max3421e_bulk_in(struct max3421e_handle_t* handle, struct max3421e_endpoint_t* endpoint, uint8_t* data, size_t len) {
    max3421e_choose_endpoint(handle, 1, endpoint);
    ssize_t total_received = 0;
    while (1) {
        uint8_t received = 0;
        uint8_t packetsize = len > endpoint->packetsize ? endpoint->packetsize : len;
        if (in_transfer(handle, VAL_HXFR_BULK_IN, endpoint, data, packetsize, &received)) {
            return -1;
        }
        if (received > 0) {
            ESP_LOGI(TAG, "BULK in ep %u len %u", endpoint->id, received);
        }
        total_received += received;
        if (received < packetsize || packetsize >= len) {
            break;
        }
        data += received;
        len -= received;
    }
    return total_received;
}

int max3421e_bulk_out(struct max3421e_handle_t* handle, struct max3421e_endpoint_t* endpoint, uint8_t* data, size_t len) {
    max3421e_choose_endpoint(handle, 0, endpoint);
    while (len > 0) {
        uint8_t packetsize = len > endpoint->packetsize ? endpoint->packetsize : len;
        ESP_LOGI(TAG, "BULK out ep %u len %u", endpoint->id, packetsize);
        if (!(xEventGroupWaitBits(handle->irq_events, BIT_HIRQ_SNDBAVIRQ, pdTRUE, pdFALSE, DEFAULT_TIMEOUT) & BIT_HIRQ_SNDBAVIRQ)) {
            ESP_LOGE(TAG, "writing timed out");
            return -1;
        }
        if (out_transfer(handle, VAL_HXFR_BULK_OUT, endpoint, REG_SNDFIFO, data, packetsize)) {
            return -1;
        }
        data += packetsize;
        len -= packetsize;
    }
    return 0;
}

static void IRAM_ATTR isr_handler(void* p) {
    struct max3421e_handle_t* handle = p;
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(handle->check_irqs_task_handle, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void check_irqs_task(void* p) {
    struct max3421e_handle_t* handle = p;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t val = read_register(handle, REG_HIRQ);
#ifdef IRQ_DEBUG
        ESP_LOG_START(TAG, PREFIX_MAGENTA);
        if (val & BIT_HIRQ_HXFRDNIRQ) {
            ESP_LOG_WRITE("HXFRDN ");
        }
        if (val & BIT_HIRQ_FRAMEIRQ) {
            ESP_LOG_WRITE("FRAME ");
        }
        if (val & BIT_HIRQ_CONDETIRQ) {
            ESP_LOG_WRITE("CONDET ");
        }
        if (val & BIT_HIRQ_SUSDNIRQ) {
            ESP_LOG_WRITE("SUSDN ");
        }
        if (val & BIT_HIRQ_SNDBAVIRQ) {
            ESP_LOG_WRITE("SNDBAV ");
        }
        if (val & BIT_HIRQ_RCVDAVIRQ) {
            ESP_LOG_WRITE("RCVDAV ");
        }
        if (val & BIT_HIRQ_RWUIRQ) {
            ESP_LOG_WRITE("RWU ");
        }
        if (val & BIT_HIRQ_BUSEVENTIRQ) {
            ESP_LOG_WRITE("BUSEVENT ");
        }
        ESP_LOG_END();
#endif
        write_register(handle, REG_HIRQ, val & ~(BIT_HIRQ_RCVDAVIRQ | BIT_HIRQ_SNDBAVIRQ));  // BIT_HIRQ_RCVDAVIRQ must be reset after reading first
        xEventGroupSetBits(handle->irq_events, val);
    }
    vTaskDelete(NULL);
}

void max3421e_select_addr(struct max3421e_handle_t* handle, uint8_t addr) {
    ESP_LOGI(TAG, "writing PERADDR 0x%02x", addr);
    write_register(handle, REG_PERADDR, addr);
}

int max3421e_control_transfer(struct max3421e_handle_t* handle, struct usb_setup_t* setup, void* data) {
    int dir_read = setup->bRequestType & 0x80;
    ESP_LOGI(TAG, "control %s type 0x%02x req 0x%02x val 0x%04x index 0x%04x", dir_read ? "in" : "out", setup->bRequestType, setup->bRequest, setup->wValue,
             setup->wIndex);

    handle->last_endpoint_used = 0;
    write_register(handle, REG_HCTL, dir_read ? BIT_HCTL_RCVTOG0 : BIT_HCTL_SNDTOG0);  // next one has to be DATA0 packet

    if (out_transfer(handle, VAL_HXFR_SETUP, &handle->ep0, REG_SUDFIFO, setup, USB_SIZE_SETUP)) {
        return -1;
    }

    write_register(handle, REG_HCTL, dir_read ? BIT_HCTL_RCVTOG1 : BIT_HCTL_SNDTOG1);  // next one has to be DATA1 packet

    ssize_t c = 0;
    if (setup->wLength) {
        if (dir_read) {
            c = max3421e_bulk_in(handle, &handle->ep0, data, setup->wLength);
            if (c < 0) {
                return -1;
            }
        } else {
            if (max3421e_bulk_out(handle, &handle->ep0, data, setup->wLength)) {
                return -1;
            }
        }
    }

    if (launch_transfer(handle, dir_read ? VAL_HXFR_HS_OUT : VAL_HXFR_HS_IN, &handle->ep0) != hrSUCCESS) {
        return -1;
    }

    if (setup->wLength && dir_read) {
        if (setup->wLength != c) {
            ESP_LOGE(TAG, "expected %u bytes but received %u", setup->wLength, c);
            return -1;
        }
    }

    ESP_LOGI(TAG, "control transfer successful");
    return 0;
}

int max3421e_get_device_descriptor(struct max3421e_handle_t* handle, struct usb_device_descriptor_t* desc) {
    struct usb_setup_t setup = {
        .bRequestType = 0x80,
        .bRequest = USB_REQUEST_GET_DESCRIPTOR,
        .wValue = USB_DESCRIPTOR_TYPE_DEVICE,
        .wIndex = 0,
        .wLength = USB_SIZE_DEVICE_DESCRIPTOR,
    };
    return max3421e_control_transfer(handle, &setup, desc);
}

int max3421e_get_config_descriptor(struct max3421e_handle_t* handle, uint16_t index, struct usb_config_descriptor_t* desc) {
    struct usb_setup_t setup = {
        .bRequestType = 0x80,
        .bRequest = USB_REQUEST_GET_DESCRIPTOR,
        .wValue = USB_DESCRIPTOR_TYPE_CONFIGURATION | index,
        .wIndex = 0,
        .wLength = USB_SIZE_CONFIG_DESCRIPTOR,
    };
    return max3421e_control_transfer(handle, &setup, desc);
}

int max3421e_is_configured(struct max3421e_handle_t* handle) {
    struct usb_setup_t setup = {
        .bRequestType = 0x80,
        .bRequest = USB_REQUEST_GET_CONFIGURATION,
        .wValue = 0,
        .wIndex = 0,
        .wLength = 1,
    };
    uint8_t data __attribute__((aligned(4))) = 0;
    if (max3421e_control_transfer(handle, &setup, &data)) {
        return -1;
    }
    return data != 0;
}

int max3421e_get_status(struct max3421e_handle_t* handle) {
    struct usb_setup_t setup = {
        .bRequestType = 0x80,
        .bRequest = USB_REQUEST_GET_STATUS,
        .wValue = 0,
        .wIndex = 0,
        .wLength = 2,
    };
    uint16_t data __attribute__((aligned(4))) = 0;
    if (max3421e_control_transfer(handle, &setup, &data)) {
        return -1;
    }
    return data;
}

int max3421e_get_full_configuration(struct max3421e_handle_t* handle, uint16_t index, uint8_t* data, size_t len) {
    struct usb_setup_t setup = {
        .bRequestType = 0x80,
        .bRequest = USB_REQUEST_GET_DESCRIPTOR,
        .wValue = USB_DESCRIPTOR_TYPE_CONFIGURATION | index,
        .wIndex = 0,
        .wLength = len,
    };

    ESP_LOGI(TAG, "starting control in transfer 0x%04x (get config descriptor)", setup.bRequest);

    handle->last_endpoint_used = 0;
    write_register(handle, REG_HCTL, BIT_HCTL_RCVTOG0);

    ESP_LOGI(TAG, "sending SETUP packet");
    if (out_transfer(handle, VAL_HXFR_SETUP, &handle->ep0, REG_SUDFIFO, &setup, USB_SIZE_SETUP)) {
        return -1;
    }

    write_register(handle, REG_HCTL, BIT_HCTL_RCVTOG1);

    ssize_t c = max3421e_bulk_in(handle, &handle->ep0, data, len);
    if (c < 0) {
        return -1;
    }

    ESP_LOGI(TAG, "sending handshake");
    if (launch_transfer(handle, VAL_HXFR_HS_OUT, &handle->ep0) != hrSUCCESS) {
        return -1;
    }

    if (c < 8) {
        ESP_LOGE(TAG, "expected at least 8 bytes but received %u", c);
        return -1;
    }
    const ssize_t total_len = ((struct usb_config_descriptor_t*)data)->wTotalLength;
    if (c != total_len) {
        ESP_LOGE(TAG, "expected %u bytes but received %u", total_len, c);
        return -1;
    }

    if (len < total_len) {
        ESP_LOGE(TAG, "buffer too small");
        return -1;
    }

    ESP_LOGI(TAG, "control transfer successful (get config descriptor)");
    return 0;
}

static int wait_for_bit(struct max3421e_handle_t* handle, uint8_t reg, uint8_t bit, uint8_t target_value) {
    ESP_LOG_START(TAG, PREFIX_BLUE);
    ESP_LOG_WRITE("waiting...");
    int c = 0;
    if (target_value) {
        target_value = bit;
    }
    vTaskDelay(pdMS_TO_TICKS(25));
    while (target_value ^ (read_register(handle, reg) & bit)) {
        if (c >= 100) {
            ESP_LOG_WRITE("\033[0;31mtimeout");
            ESP_LOG_END();
            return -1;
        }
        ++c;
        ESP_LOG_WRITE(".");
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    ESP_LOG_WRITE("done");
    ESP_LOG_END();
    return 0;
}

static int probe_bus(struct max3421e_handle_t* handle) {
    ESP_LOGI(TAG, "sample bus");
    write_register(handle, REG_HCTL, BIT_HCTL_SAMPLEBUS);
    if (wait_for_bit(handle, REG_HCTL, BIT_HCTL_SAMPLEBUS, 1)) {
        return -1;
    }

    handle->highspeed = (read_register(handle, REG_MODE) & BIT_MODE_LOWSPEED) ? 0 : 1;

    uint8_t hrsl = read_register(handle, REG_HRSL);
    switch (hrsl & (BIT_HRSL_JSTATUS | BIT_HRSL_KSTATUS)) {
        case BIT_HRSL_JSTATUS | BIT_HRSL_KSTATUS:
            ESP_LOGE(TAG, "illegal state");
            return -1;

        case BIT_HRSL_JSTATUS:
            ESP_LOGI(TAG, "in JSTATUS, %s", handle->highspeed ? "highspeed" : "lowspeed");
            break;

        case BIT_HRSL_KSTATUS:
            ESP_LOGI(TAG, "in KSTATUS, %s", handle->highspeed ? "lowspeed" : "highspeed");
            handle->highspeed = !handle->highspeed;
            break;

        case 0:
            ESP_LOGE(TAG, "no device present");
            return 0;
    }

    write_register(handle, REG_CPUCTL, BIT_CPUCTL_IE);  // necessary before write to mode

    write_register(handle, REG_MODE, BIT_MODE_DPPULLDN | BIT_MODE_DMPULLDN | BIT_MODE_SOFKAENAB | BIT_MODE_HOST | (handle->highspeed ? 0 : BIT_MODE_LOWSPEED));

    ESP_LOGI(TAG, "reset bus");
    write_register(handle, REG_HCTL, BIT_HCTL_BUSRST);
    if (wait_for_bit(handle, REG_HCTL, BIT_HCTL_BUSRST, 0)) {
        return -1;
    }

    return handle->highspeed ? 2 : 1;
}

int max3421e_init(spi_t spi, gpio_num_t int_gpio, gpio_num_t reset_gpio, struct max3421e_handle_t* handle) {
    ESP_LOGI(TAG, "starting up");
    memset(handle, 0, sizeof(struct max3421e_handle_t));
    handle->spi = spi;
    handle->irq_events = xEventGroupCreate();
    handle->spi_mutex = xSemaphoreCreateMutex();
    xTaskCreate(check_irqs_task, "check_irqs_task", 2048, handle, 2, &handle->check_irqs_task_handle);

#ifndef USE_SPI_DEVICE
    spi_config_t spi_config = {
        .interface = {.val = SPI_DEFAULT_INTERFACE},
        .intr_enable = {.val = 0},
        .mode = SPI_MASTER_MODE,
        .clk_div = SPI_20MHz_DIV,
        .event_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_init(spi, &spi_config));
#endif

    ESP_ERROR_CHECK(gpio_set_direction(int_gpio, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(int_gpio, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_intr_type(int_gpio, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0 /* no_use */));
    ESP_ERROR_CHECK(gpio_isr_handler_add(int_gpio, isr_handler, handle));

    ESP_LOGI(TAG, "resetting");
    ESP_ERROR_CHECK(gpio_set_direction(reset_gpio, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(reset_gpio, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(reset_gpio, 1));

    write_register(handle, REG_PINCTL, BIT_PINCTL_FDUPSPI);

    write_register(handle, REG_USBCTL, BIT_USBCTL_CHIPRES);
    write_register(handle, REG_USBCTL, 0);
    if (wait_for_bit(handle, REG_USBIRQ, BIT_USBIRQ_OSCOKIRQ, 1)) {
        return -1;
    }
    ESP_LOGI(TAG, "max3421e version 0x%02x", read_register(handle, REG_REVISION));

    write_register(handle, REG_MODE, BIT_MODE_DPPULLDN | BIT_MODE_DMPULLDN | BIT_MODE_HOST);  // TODO lowspeed

    write_register(handle, REG_USBIEN, 0);
    write_register(handle, REG_USBIRQ, 0xff);

    write_register(handle, REG_HIEN,
                   BIT_HIEN_HXFRDNIE        // Host Transfer Done Interrupt Enable
                       | BIT_HIEN_CONDETIE  // Peripheral Connect/Disconnect Interrupt Enable
                                            //| BIT_HIEN_FRAMEIE     // Frame Generator Interrupt Enable
                                            //| BIT_HIEN_SUSDNIE     // Suspend operation Done IE
                       | BIT_HIEN_SNDBAVIE  // Send Buffer Available Interrupt Enable
                       | BIT_HIEN_RCVDAVIE  // Receive FIFO Data Available Interrupt Enable
                                            //| BIT_HIEN_RWUIE       // Remote Wakeup Interrupt Enable
                                            //| BIT_HIEN_BUSEVENTIE  // Enable the BUSEVENTIRQ
    );
    write_register(handle, REG_HIRQ, ~(BIT_HIRQ_RCVDAVIRQ | BIT_HIRQ_SNDBAVIRQ));
    return 0;
}

int max3421e_setup(struct max3421e_handle_t* handle) {
    handle->highspeed = 0;
    int state = probe_bus(handle);
    if (state < 0) {  // error
        return -1;
    }
    if (state == 0) {  // no device present
        return 1;
    }
    handle->last_endpoint_used = 0xff;
    handle->ep0.id = 0;
    handle->ep0.datatoggles = 0;

    write_register(handle, REG_USBIRQ, 0xff);
    write_register(handle, REG_HIRQ, 0xff);
    write_register(handle, REG_PERADDR, 0);

    uint8_t buf[8] __attribute__((aligned(4)));
    handle->ep0.packetsize = sizeof(buf);
    {
        struct usb_setup_t setup = {
            .bRequestType = 0x80,
            .bRequest = USB_REQUEST_GET_DESCRIPTOR,
            .wValue = USB_DESCRIPTOR_TYPE_DEVICE,
            .wIndex = 0,
            .wLength = sizeof(buf),
        };
        if (max3421e_control_transfer(handle, &setup, buf)) {
            return -1;
        }
    }
    handle->ep0.packetsize = buf[7];
    // TODO
    /* if (handle->packetsize != 8 && handle->packetsize != 16 && handle->packetsize != 32 && handle->packetsize != 64) { */
    /*     ESP_LOGE(TAG, "received invalid maxPacketSize of %u", handle->packetsize); */
    /*     return -1; */
    /* } */

    {
        struct usb_setup_t setup = {
            .bRequestType = 0,
            .bRequest = USB_REQUEST_SET_ADDRESS,
            .wValue = 1,  // TODO
            .wIndex = 0,
            .wLength = 0,
        };
        if (max3421e_control_transfer(handle, &setup, NULL)) {
            return -1;
        }
    }

    return 0;
}

int max3421e_select_configuration(struct max3421e_handle_t* handle, uint16_t configuration) {
    struct usb_setup_t setup = {
        .bRequestType = 0,
        .bRequest = USB_REQUEST_SET_CONFIGURATION,
        .wValue = configuration,
        .wIndex = 0,
        .wLength = 0,
    };
    return max3421e_control_transfer(handle, &setup, NULL);
}

int max3421e_wait_for_connection(struct max3421e_handle_t* handle, TickType_t timeout) {
    return !(xEventGroupWaitBits(handle->irq_events, BIT_HIRQ_CONDETIRQ, pdTRUE, pdFALSE, timeout) & BIT_HIRQ_CONDETIRQ);
}

void usb_log_device_descriptor(const char* tag, struct usb_device_descriptor_t* desc) {
    ESP_LOGI(tag, "bLength: 0x%02x", desc->bLength);
    ESP_LOGI(tag, "bDescriptorType: 0x%02x", desc->bDescriptorType);
    ESP_LOGI(tag, "bcdUSB: 0x%04x", desc->bcdUSB);
    ESP_LOGI(tag, "bDeviceClass: 0x%02x", desc->bDeviceClass);
    ESP_LOGI(tag, "bDeviceSubClass: 0x%02x", desc->bDeviceSubClass);
    ESP_LOGI(tag, "bDeviceProtocol: 0x%02x", desc->bDeviceProtocol);
    ESP_LOGI(tag, "bMaxPacketSize0: 0x%02x", desc->bMaxPacketSize0);
    ESP_LOGI(tag, "idVendor: 0x%04x", desc->idVendor);
    ESP_LOGI(tag, "idProduct: 0x%04x", desc->idProduct);
    ESP_LOGI(tag, "bcdDevice: 0x%04x", desc->bcdDevice);
    ESP_LOGI(tag, "iManufacturer: 0x%02x", desc->iManufacturer);
    ESP_LOGI(tag, "iProduct: 0x%02x", desc->iProduct);
    ESP_LOGI(tag, "iSerialNumber: 0x%02x", desc->iSerialNumber);
    ESP_LOGI(tag, "bNumConfigurations: 0x%02x", desc->bNumConfigurations);
}

void usb_log_config_descriptor(const char* tag, struct usb_config_descriptor_t* desc) {
    ESP_LOGI(tag, "bLength: 0x%02x", desc->bLength);
    ESP_LOGI(tag, "bDescriptorType: 0x%02x", desc->bDescriptorType);
    ESP_LOGI(tag, "wTotalLength: 0x%04x", desc->wTotalLength);
    ESP_LOGI(tag, "bNumInterfaces: 0x%02x", desc->bNumInterfaces);
    ESP_LOGI(tag, "bConfigurationValue: 0x%02x", desc->bConfigurationValue);
    ESP_LOGI(tag, "iConfiguration: 0x%02x", desc->iConfiguration);
    ESP_LOGI(tag, "bmAttributes: 0x%02x", desc->bmAttributes);
    ESP_LOGI(tag, "bMaxPower: 0x%02x", desc->bMaxPower);
}

void usb_log_interface_descriptor(const char* tag, struct usb_interface_descriptor_t* desc) {
    ESP_LOGI(tag, "bLength: 0x%02x", desc->bLength);
    ESP_LOGI(tag, "bDescriptorType: 0x%02x", desc->bDescriptorType);
    ESP_LOGI(tag, "bInterfaceNumber: 0x%02x", desc->bInterfaceNumber);
    ESP_LOGI(tag, "bAlternateSetting: 0x%02x", desc->bAlternateSetting);
    ESP_LOGI(tag, "bNumEndpoints: 0x%02x", desc->bNumEndpoints);
    ESP_LOGI(tag, "bInterfaceClass: 0x%02x", desc->bInterfaceClass);
    ESP_LOGI(tag, "bInterfaceSubClass: 0x%02x", desc->bInterfaceSubClass);
    ESP_LOGI(tag, "bInterfaceProtocol: 0x%02x", desc->bInterfaceProtocol);
    ESP_LOGI(tag, "iInterface: 0x%02x", desc->iInterface);
}

void usb_log_endpoint_descriptor(const char* tag, struct usb_endpoint_descriptor_t* desc) {
    ESP_LOGI(tag, "bLength: 0x%02x", desc->bLength);
    ESP_LOGI(tag, "bDescriptorType: 0x%02x", desc->bDescriptorType);
    ESP_LOGI(tag, "bEndpointAddress: 0x%02x", desc->bEndpointAddress);
    ESP_LOGI(tag, "bmAttributes: 0x%02x", desc->bmAttributes);
    ESP_LOGI(tag, "wMaxPacketSize: 0x%04x", desc->wMaxPacketSize);
    ESP_LOGI(tag, "bInterval: 0x%02x", desc->bInterval);
}
