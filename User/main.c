#include <stdlib.h>

#include "arm_pose.h"
#include "bsp_beep.h"
#include "bsp_rcc.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "user_def.h"

#define BUS_SERVO_DEBUG

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
    // 动作1
    bsp_uart1_send_string("Move 1!\r\n");
    arm_frame_t frame = {
        .pose = {.joint = {1600, 1600, 1600, 1600, 1600, 1600}},
        .time = 1000,
    };
    arm_play_frame(&frame);
    bsp_delay_ms(2000);
    // 回正
    bsp_uart1_send_string("Move 2!\r\n");
    arm_set_home(1000);
    while (1) {
        bsp_delay_ms(2000);
    }
}
