#ifndef TELNET_H
#define TELNET_H

#include <freertos/FreeRTOS.h>
//
#include <driver/gpio.h>
#include <driver/uart.h>

void telnet_start();
void telnet_start_with_uart(uart_port_t uart, gpio_num_t dtr_pin /* used for GPIO0 */, gpio_num_t rts_pin /* used for switching on/off */);
void telnet_stop();

#endif
