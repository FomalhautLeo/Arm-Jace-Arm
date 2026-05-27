#include <stdlib.h>

#include "bsp_rcc.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "user_def.h"

static void delay(volatile uint32_t count) {
    while (count--) {
    }
}

int main(void) {
    // 时钟初始化
    bsp_rcc_init();
    // 系统时钟初始化
    bsp_systick_init();
    // USART1 串口初始化
    bsp_uart1_init(DEFAULT_BAUDRATE);
    // USART3 串口初始化
    bsp_uart3_init(DEFAULT_BAUDRATE);

    // 调试信息
    bsp_uart1_send_string("Hello STM!\r\n");
    // // 发送舵机信息
		bsp_uart1_send_string("Move 1!\r\n");
		bsp_uart3_send_string("#000P1300T1000!");
		bsp_delay_ms(2000);
		bsp_uart1_send_string("Move 2!\r\n");
		bsp_uart3_send_string("#000P1700T1000!");
		bsp_delay_ms(2000);
    while (1) {
				bsp_delay_ms(2000);
    }
}
