#ifndef TELNET_H
#define TELNET_H

#define TELNET_UART_SUPPORT

#include <freertos/FreeRTOS.h>

#ifdef TELNET_UART_SUPPORT
#include <driver/gpio.h>
#include <driver/uart.h>
void telnet_start_with_uart(uart_port_t uart, gpio_num_t dtr_pin /* used for GPIO0 */, gpio_num_t rts_pin /* used for switching on/off */);
#endif

void telnet_start();
void telnet_stop();

#endif
