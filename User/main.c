#include <stdlib.h>

#include "bsp_beep.h"
#include "bsp_rcc.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "bus_servo.h"
#include "user_def.h"

int main(void) {
    // 时钟初始化
    bsp_rcc_init();
    // 系统时钟初始化
    bsp_systick_init();
    // 蜂鸣器初始化
    bsp_beep_init();

    // USART1 串口初始化
    bsp_uart1_init(DEFAULT_BAUDRATE);
    // USART3 串口初始化
    bsp_uart3_init(DEFAULT_BAUDRATE);

    // 蜂鸣器响，初始化完成
    bsp_beep(200, 3);
    // 调试信息
    bsp_uart1_send_string("Hello STM!\r\n");
    // // 发送舵机信息
    bsp_uart1_send_string("Move 1!\r\n");
    bus_servo_cmd_t cmds[] = {
        {0, 1700},
        {1, 1700},
    };
    bus_servo_move_group(cmds, 2, 1000);
    bsp_delay_ms(2000);
    bsp_uart1_send_string("Move 2!\r\n");
    cmds[0].pos = 1500;
    cmds[1].pos = 1500;
    bus_servo_move_group(cmds, 2, 1000);
    while (1) {
        bsp_delay_ms(2000);
    }
}
