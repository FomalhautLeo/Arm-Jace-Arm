#include "bsp_rcc.h"
#include "bsp_uart.h"
#include "user_def.h"

int main(void) {
    // 时钟初始化
    bsp_rcc_init();
    // USART1 串口初始化
    bsp_uart1_init(DEFAULT_BAUDRATE);

    bsp_uart1_send_string("Hello STM!\r\n");

    while (1) {
    }
}
