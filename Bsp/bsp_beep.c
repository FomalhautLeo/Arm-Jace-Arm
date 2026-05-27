#include "bsp_beep.h"

#include "bsp_systick.h"
#include "stm32f10x_rcc.h"

static uint8_t beep_status = 0;

void bsp_beep_init(void) {
    // 开启GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // 引脚配置
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Pin = GPIO_Pin_5;
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio_init);
}

void bsp_beep_on(void) {
    GPIO_SetBits(GPIOB, BEEP_PIN);
    beep_status = 1;
}
void bsp_beep_off(void) {
    GPIO_ResetBits(GPIOB, BEEP_PIN);
    beep_status = 0;
}
void bsp_beep_toggle(void) {
    if (beep_status) {
        bsp_beep_off();
    } else {
        bsp_beep_on();
    }
}

// 蜂鸣器持续响 ms count 次
void bsp_beep(uint32_t ms, uint32_t count) {
    while (count--) {
        bsp_beep_on();
        bsp_delay_ms(ms);
        bsp_beep_off();
        bsp_delay_ms(ms);
    }
}
