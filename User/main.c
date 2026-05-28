#include <stdlib.h>

#include "arm_action.h"
#include "bsp_beep.h"
#include "bsp_nvic.h"
#include "bsp_rcc.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "command_parser.h"
#include "uart_frame.h"
#include "user_debug.h"
#include "user_def.h"

static void test_joint(uint8_t joint_id, uint16_t pos) {
    arm_pose_t pose = {.joint = {1500, 1500, 1500, 1500, 1500, 1500}};

    if (joint_id >= SERVO_COUNT) return;

    pose.joint[joint_id] = pos;
    arm_set_pose(&pose, 800);
}

int main(void) {
    // 时钟初始化
    bsp_rcc_init();
    // 中断初始化
    bsp_nvic_init();
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
    // 开始信息
    debug_println("Hello Servo!");

    // 先回正
    arm_set_home(1000);

    char cmd[UART_COMMAND_BUFFER_LENGTH];
    char servo_resp[UART_COMMAND_BUFFER_LENGTH];

    while (1) {
        // 接收串口调试数据
        if (uart_frame_is_ready(UART_FRAME_USER)) {
            uart_frame_get(UART_FRAME_USER, cmd, sizeof(cmd));
            command_parser_handle(cmd);
        }
        // 接收舵机回发数据
        if (uart_frame_is_ready(UART_FRAME_SERVO)) {
            uart_frame_get(UART_FRAME_SERVO, servo_resp, sizeof(servo_resp));
            debug_print("[SERVO_RX] ");
            debug_println(servo_resp);
        }
    }
}
