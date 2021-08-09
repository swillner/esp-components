#include "max3421e.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
//
#include <esp_log.h>
#include <string.h>

#include "max3421e_internal.h"

// TODO timeout (only for chip not responding, max3421e handles USB timeouts)
// TODO what about failing spi?
// TODO save data toggles

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

static const char* TAG = "max3421e";

static inline void spi_transfer(struct max3421e_handler_data_t* data,
#ifdef USE_SPI_DEVICE
                                spi_transaction_t* trans
#else
                                spi_trans_t* trans
#endif
) {
    xSemaphoreTake(data->spi_mutex, portMAX_DELAY);
#ifdef USE_SPI_DEVICE
    spi_device_transmit(data->spi, trans);
#else
    spi_trans(data->spi, trans);
#endif
    xSemaphoreGive(data->spi_mutex);
}

static uint8_t IRAM_ATTR read_register(struct max3421e_handler_data_t* data, uint8_t reg) {
    uint16_t cmd = (reg << 3) | CMD_READ;
#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .flags = SPI_TRANS_USE_RXDATA,
        .rxlength = 8,
    };
    spi_transfer(data, &trans);
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
    spi_transfer(data, &trans);
    return res;
#endif
}

static void IRAM_ATTR write_register(struct max3421e_handler_data_t* data, uint8_t reg, uint8_t val) {
    uint16_t cmd = (reg << 3) | CMD_WRITE;
#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_data = {val, 0, 0, 0},
    };
    spi_transfer(data, &trans);
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
    spi_transfer(data, &trans);
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

static enum max3421e_result_t do_transfer(struct max3421e_handler_data_t* data, uint8_t type, uint8_t endpoint) {
    ESP_LOGI(TAG, "transfer type 0x%02x ep 0x%02x", type, endpoint);
    xEventGroupClearBits(data->irq_events, BIT_HIRQ_HXFRDNIRQ);
    for (int i = 0; i < 10000000; ++i) {  // TODO NAK tries
        write_register(data, REG_HXFR, type | endpoint);
        xEventGroupWaitBits(data->irq_events, BIT_HIRQ_HXFRDNIRQ, pdTRUE, pdFALSE, portMAX_DELAY);
        uint8_t hrsl = read_register(data, REG_HRSL);
        enum max3421e_result_t res = hrsl & MASK_HRSL_HRSLT3;
        switch (res) {
            case hrSUCCESS:
                ESP_LOGI(TAG, "transfer successful");
                return res;

            case hrNAK:
                break;

            case hrTOGERR:
                ESP_LOGI(TAG, "retoggle");
                write_register(data, REG_HCTL, (hrsl & BIT_HRSL_RCVTOGRD) ? BIT_HCTL_RCVTOG0 : BIT_HCTL_RCVTOG1);
                break;

            default:
                ESP_LOGE(TAG, "transfer failed with %s", get_result_description(res));
                return res;
        }
    }
    ESP_LOGE(TAG, "transfer timeout");
    return hrTIMEOUT;
}

static int in_transfer(struct max3421e_handler_data_t* data, uint8_t type, uint8_t endpoint, uint32_t* buf, uint8_t size, uint8_t* recv_size) {
    if (do_transfer(data, type, endpoint) != hrSUCCESS) {
        return 1;
    }
    if (!(xEventGroupGetBits(data->irq_events) & BIT_HIRQ_RCVDAVIRQ)) {
        ESP_LOGE(TAG, "RCVDAVIRQ should have been set (0x%02x)", xEventGroupGetBits(data->irq_events));
        return 1;
    }
    *recv_size = read_register(data, REG_RCVBC);
    ESP_LOGI(TAG, "received %u bytes", *recv_size);
    // TODO what if size is not multiple of 4?
    // TODO what if size > 64
    uint16_t cmd = (REG_RCVFIFO << 3) | CMD_READ;

#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .rxlength = 8 * (*recv_size),
        .rx_buffer = buf,
    };
    spi_transfer(data, &trans);
#else
    spi_trans_t trans = {.cmd = &cmd,
                         .addr = NULL,
                         .mosi = NULL,
                         .miso = buf,
                         .bits = {
                             .cmd = 8,
                             .addr = 0,
                             .mosi = 0,
                             .miso = 8 * (*recv_size),
                         }};
    spi_transfer(data, &trans);
#endif

    xEventGroupClearBits(data->irq_events, BIT_HIRQ_RCVDAVIRQ);
    return 0;
}

static int out_transfer(struct max3421e_handler_data_t* data, uint8_t type, uint8_t endpoint, uint8_t fifo_reg, void* buf, uint8_t size) {
    // TODO what if size is not multiple of 4?
    // TODO what if size > 64
    ESP_LOGI(TAG, "sending %u bytes", size);
    uint16_t cmd = (fifo_reg << 3) | CMD_WRITE;
#ifdef USE_SPI_DEVICE
    spi_transaction_t trans = {
        .cmd = cmd,
        .length = 8 * size,
        .tx_buffer = buf,
    };
    spi_transfer(data, &trans);
#else
    spi_trans_t trans = {.cmd = &cmd,
                         .addr = NULL,
                         .mosi = buf,
                         .miso = NULL,
                         .bits = {
                             .cmd = 8,
                             .addr = 0,
                             .mosi = 8 * size,
                             .miso = 0,
                         }};
    spi_transfer(data, &trans);
#endif
    return do_transfer(data, type, endpoint) != hrSUCCESS;
}

static void IRAM_ATTR isr_handler(void* data_p) {
    struct max3421e_handler_data_t* data = data_p;
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(data->check_irqs_task_handle, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void check_irqs_task(void* data_p) {
    struct max3421e_handler_data_t* data = data_p;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t val = read_register(data, REG_HIRQ);
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
        write_register(data, REG_HIRQ, val);
        xEventGroupSetBits(data->irq_events, val);
    }
    vTaskDelete(NULL);
}

int max3421e_control_transfer(struct max3421e_handler_data_t* data, uint8_t addr, uint8_t endpoint, struct usb_setup_t* setup, void* buf) {
    int dir_read = setup->bRequestType & 0x80;

    ESP_LOGI(TAG, "writing PERADDR 0x%02x", addr);
    write_register(data, REG_PERADDR, addr);

    write_register(data, REG_HCTL, BIT_HCTL_RCVTOG0);

    ESP_LOGI(TAG, "sending SETUP packet");
    if (out_transfer(data, VAL_HXFR_SETUP, endpoint, REG_SUDFIFO, setup, sizeof(struct usb_setup_t))) {
        return 1;
    }

    write_register(data, REG_HCTL, BIT_HCTL_RCVTOG1);

    if (setup->wLength) {
        if (dir_read) {
            ESP_LOGI(TAG, "receiving data");
            uint8_t received_size = 0;
            if (in_transfer(data, VAL_HXFR_BULK_IN, endpoint, buf, setup->wLength, &received_size)) {
                return 1;
            }
            if (received_size != setup->wLength) {
                return 1;
            }
        } else {
            ESP_LOGI(TAG, "sending data");
            if (out_transfer(data, VAL_HXFR_BULK_OUT, endpoint, REG_SNDFIFO, buf, setup->wLength)) {
                return 1;
            }
        }
    }

    write_register(data, REG_HCTL, BIT_HCTL_RCVTOG1);

    ESP_LOGI(TAG, "sending handshake");
    if (do_transfer(data, dir_read ? VAL_HXFR_HS_OUT : VAL_HXFR_HS_IN, 0) != hrSUCCESS) {
        return 1;
    }

    return 0;
}

int max3421e_get_device_descriptor(struct max3421e_handler_data_t* data, struct usb_device_descriptor_t* buf) {
    struct usb_setup_t setup = {
        .bRequestType = 0x80,
        .bRequest = USB_REQUEST_GET_DESCRIPTOR,
        .wValue = USB_DESCRIPTOR_TYPE_DEVICE,
        .wIndex = 0,
        .wLength = sizeof(struct usb_device_descriptor_t),
    };
    return max3421e_control_transfer(data, 0, 0, &setup, buf);
}

static int wait_for_bit(struct max3421e_handler_data_t* data, uint8_t reg, uint8_t bit, uint8_t target_value) {
    ESP_LOG_START(TAG, PREFIX_BLUE);
    ESP_LOG_WRITE("waiting...");
    int c = 0;
    if (target_value) {
        target_value = bit;
    }
    while (target_value ^ (read_register(data, reg) & bit)) {
        if (c >= 100) {
            ESP_LOG_WRITE("\033[0;31mtimeout");
            ESP_LOG_END();
            return 1;
        }
        ++c;
        ESP_LOG_WRITE(".");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOG_WRITE("done");
    ESP_LOG_END();
    return 0;
}

int max3421e_init(spi_t spi, gpio_num_t int_gpio, gpio_num_t reset_gpio, struct max3421e_handler_data_t* data) {
    ESP_LOGI(TAG, "starting up");
    memset(data, 0, sizeof(struct max3421e_handler_data_t));
    data->spi = spi;
    data->irq_events = xEventGroupCreate();
    data->spi_mutex = xSemaphoreCreateMutex();
    xTaskCreate(check_irqs_task, "check_irqs_task", 2048, data, 2, &data->check_irqs_task_handle);

#ifndef USE_SPI_DEVICE
    spi_config_t spi_config = {
        .interface = {.val = SPI_DEFAULT_INTERFACE},
        /*         .interface = */
        /*             { */
        /*                 .cpol = SPI_CPOL_LOW, */
        /*                 .cpha = SPI_CPHA_LOW, */
        /* //#define BIT_MSB_DEF */
        /* #ifdef BIT_MSB_DEF */
        /*                 .bit_tx_order = SPI_BIT_ORDER_MSB_FIRST, */
        /*                 .bit_rx_order = SPI_BIT_ORDER_MSB_FIRST, */
        /* #else */
        /*                 .bit_tx_order = SPI_BIT_ORDER_LSB_FIRST, */
        /*                 .bit_rx_order = SPI_BIT_ORDER_LSB_FIRST, */
        /* #endif */
        /* //#define BYTE_MSB_DEF */
        /* #ifdef BYTE_MSB_DEF */
        /*                 .byte_tx_order = SPI_BYTE_ORDER_MSB_FIRST, */
        /*                 .byte_rx_order = SPI_BYTE_ORDER_MSB_FIRST, */
        /* #else */
        /*                 .byte_tx_order = SPI_BYTE_ORDER_LSB_FIRST, */
        /*                 .byte_rx_order = SPI_BYTE_ORDER_LSB_FIRST, */
        /* #endif */
        /*                 .mosi_en = 1, */
        /*                 .miso_en = 1, */
        /*                 .cs_en = 1, */
        /*             }, */
        //.intr_enable = {.val = SPI_MASTER_DEFAULT_INTR_ENABLE},
        .intr_enable = {.val = 0},
        .mode = SPI_MASTER_MODE,
        .clk_div = SPI_2MHz_DIV,  // TODO
        .event_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_init(spi, &spi_config));
#endif

    ESP_ERROR_CHECK(gpio_set_direction(int_gpio, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(int_gpio, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_intr_type(int_gpio, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0 /* no_use */));
    ESP_ERROR_CHECK(gpio_isr_handler_add(int_gpio, isr_handler, data));
    ESP_LOGI(TAG, "isr_handler installed");

    ESP_ERROR_CHECK(gpio_set_direction(reset_gpio, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(reset_gpio, 1));

    ESP_LOGI(TAG, "writing to max3421e");
    write_register(data, REG_PINCTL, BIT_PINCTL_FDUPSPI);

    /* reset and wait for OSCOKIRQ */
    ESP_LOGI(TAG, "resetting");
    write_register(data, REG_USBCTL, BIT_USBCTL_CHIPRES);
    write_register(data, REG_USBCTL, 0);
    if (wait_for_bit(data, REG_USBIRQ, BIT_USBIRQ_OSCOKIRQ, 1)) {
        return 1;
    }
    ESP_LOGI(TAG, "max3421e version 0x%02x", read_register(data, REG_REVISION));

    write_register(data, REG_MODE, BIT_MODE_DPPULLDN | BIT_MODE_DMPULLDN | BIT_MODE_HOST);

    write_register(data, REG_USBIEN, 0);
    write_register(data, REG_HIEN, 0);

    ESP_LOGI(TAG, "sample bus");
    write_register(data, REG_HCTL, BIT_HCTL_SAMPLEBUS);
    if (wait_for_bit(data, REG_HCTL, BIT_HCTL_SAMPLEBUS, 1)) {
        return 1;
    }

    uint8_t hrsl = read_register(data, REG_HRSL);
    ESP_LOGI(TAG, "sampled JSTATUS %d", !!(hrsl & BIT_HRSL_JSTATUS));
    ESP_LOGI(TAG, "sampled KSTATUS %d", !!(hrsl & BIT_HRSL_KSTATUS));

    write_register(data, REG_CPUCTL, BIT_CPUCTL_IE);  // necessary bevore write to mode

    // if ((hrsl & BIT_HRSL_JSTATUS) || (hrsl & BIT_HRSL_KSTATUS)) {
    write_register(data, REG_MODE,
                   BIT_MODE_DPPULLDN | BIT_MODE_DMPULLDN
                       | BIT_MODE_SOFKAENAB
                       //| BIT_MODE_LOWSPEED
                       | BIT_MODE_HOST);

    ESP_LOGI(TAG, "reset bus");
    write_register(data, REG_HCTL, BIT_HCTL_BUSRST);
    if (wait_for_bit(data, REG_HCTL, BIT_HCTL_BUSRST, 0)) {
        return 1;
    }
    //}

    write_register(data, REG_USBIRQ, 0xff);
    write_register(data, REG_HIRQ, 0xff);
    write_register(data, REG_HIEN,
                   0                        //
                       | BIT_HIEN_HXFRDNIE  // Host Transfer Done Interrupt Enable
                       //| BIT_HIEN_FRAMEIE     // Frame Generator Interrupt Enable
                       | BIT_HIEN_CONDETIE  // Peripheral Connect/Disconnect Interrupt Enable
                                            //| BIT_HIEN_SUSDNIE   // Suspend operation Done IE
                                            //| BIT_HIEN_SNDBAVIE  // Send Buffer Available Interrupt Enable
                                            //| BIT_HIEN_RCVDAVIE    // Receive FIFO Data Available Interrupt Enable
                                            //| BIT_HIEN_RWUIE       // Remote Wakeup Interrupt Enable
                                            //| BIT_HIEN_BUSEVENTIE  // Enable the BUSEVENTIRQ
    );
    // write_register(data, REG_USBIEN, 0xff);
    // write_register(data, REG_HIEN, 0xff);
    return 0;
}

int max3421e_wait_for_connection(struct max3421e_handler_data_t* data) {
    xEventGroupWaitBits(data->irq_events, BIT_HIRQ_CONDETIRQ, pdTRUE, pdFALSE, portMAX_DELAY);
    // TODO optional timeout
    return 0;
}
