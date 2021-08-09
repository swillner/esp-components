#ifndef STATUS_H
#define STATUS_H

#include <driver/gpio.h>
#include <stdint.h>

#define STATUS_LED_OFF 0
#define STATUS_LED_SLOW 1
#define STATUS_LED_MID 2
#define STATUS_LED_FAST 3

void status_init(gpio_num_t gpio, uint8_t status);
void status_set(uint8_t status);

#endif
