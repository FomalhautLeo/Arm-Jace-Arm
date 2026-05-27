#ifndef __BSP_BEEP_H__
#define __BSP_BEEP_H__

#include "stm32f10x_gpio.h"

#define BEEP_PIN GPIO_Pin_5

void bsp_beep_init(void);
void bsp_beep_on(void);
void bsp_beep_off(void);
void bsp_beep_toggle(void);
void bsp_beep(uint32_t time, uint32_t count);

#endif  // __BSP_BEEP_H__
