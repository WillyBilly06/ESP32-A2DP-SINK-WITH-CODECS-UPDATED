#ifndef FAKE_GPIO
#define FAKE_GPIO
#include <stdint.h>
typedef int gpio_num_t;
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP_ENABLE 1
typedef struct {int mode,pull_up_en;uint64_t pin_bit_mask;} gpio_config_t;
int gpio_config(const gpio_config_t*);int gpio_get_level(gpio_num_t);
#endif
