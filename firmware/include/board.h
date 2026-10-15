#ifndef BOARD_H
#define BOARD_H
#include "stm32h7xx.h"
#include <stdint.h>
void board_init(void);
void board_gpio(GPIO_TypeDef *port, unsigned pin, unsigned mode, unsigned af);
void board_led(unsigned green, unsigned blue);
unsigned board_led_state(void);
uint32_t board_reset_flags(void);
uint32_t board_lock(void);
void board_unlock(uint32_t key);
void board_delay_us(uint32_t us);
#endif
