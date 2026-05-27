#include "bsp_systick.h"

#include "stm32f10x.h"

// 系统毫秒计数，注意防止内存优化
static volatile uint32_t system_milli = 0;

void bsp_systick_init(void) {
    // 1秒分成1000份，保证1ms触发一次中断
    SysTick_Config(SystemCoreClock / 1000);
}

void bsp_systick_tick(void) {
    // 系统计数步进
    system_milli++;
}

void bsp_delay_ms(uint32_t ms) {
    // 需要等到的计数
    uint32_t start = system_milli;
    // 忙等，减法处理溢出
    while ((uint32_t)(system_milli - start) < ms) {
    }
}

uint32_t bsp_millis(void) { return system_milli; }
