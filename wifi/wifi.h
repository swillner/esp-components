#ifndef WIFI_H
#define WIFI_H

#include <freertos/FreeRTOS.h>

int wifi_is_access_point();
int wifi_is_connected();
int wifi_wait_for_connection(TickType_t wait_time);
uint32_t wifi_get_local_ip();
void wifi_connect();
void wifi_console_register();
void wifi_init();
void wifi_start_access_point();

#endif
