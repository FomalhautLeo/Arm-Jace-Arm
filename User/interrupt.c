
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "stm32f10x_usart.h"
#include "uart_frame.h"

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
    uart_frame_on_byte(UART_FRAME_USER, (uint8_t)USART_ReceiveData(USART1));
}

// USART3 中断处理函数
void USART3_IRQHandler(void) {
    // 先判断中断类型
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != SET) return;
    // 写入新数据
    uart_frame_on_byte(UART_FRAME_SERVO, (uint8_t)USART_ReceiveData(USART3));
}
