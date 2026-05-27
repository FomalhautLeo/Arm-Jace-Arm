#ifndef __BSP_SYSTICK_H__
#define __BSP_SYSTICK_H__

#include <stdint.h>

void bsp_systick_init(void);
void bsp_systick_tick(void);
void bsp_delay_ms(uint32_t ms);
uint32_t bsp_millis(void);

#endif  // __BSP_SYSTICK_H__
