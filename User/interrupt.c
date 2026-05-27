
#include "bsp_systick.h"
#include "stm32f10x_usart.h"
#include "uart_command.h"

// SysTick 中断处理函数
void SysTick_Handler(void) {
    // tick 步进
    bsp_systick_tick();
}

// USART1 中断处理函数
void USART1_IRQHandler(void) {
    // 先判断中断类型
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != SET) return;
    // 写入新数据
    uart_command_on_byte((uint8_t)USART_ReceiveData(USART1));
}
