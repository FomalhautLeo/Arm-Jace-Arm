#include <stdlib.h>

#include "arm_action.h"
#include "bsp_beep.h"
#include "bsp_nvic.h"
#include "bsp_rcc.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "uart_command.h"
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
    // 调试信息
    bsp_uart1_send_string("Hello STM!\r\n");
    /*
    // 动作组
    bsp_uart1_send_string("Move 1!\r\n");
    const arm_frame_t frames[] = {
        {
            .pose = {.joint = {1400, 1400, 1400, 1400, 1400, 1400}},
            .time = 1000,
        },
        {
            .pose = {.joint = {1600, 1600, 1600, 1600, 1600, 1600}},
            .time = 1000,
        },
        {
            .pose = {.joint = {1400, 1400, 1400, 1400, 1400, 1400}},
            .time = 1000,
        },
    };

    arm_action_t action = {
        .frames = frames,
        .frame_count = 3,
    };
    // 执行动作组
    arm_play_action(&action);
    bsp_delay_ms(2000);
    // 回正
    bsp_uart1_send_string("Move 2!\r\n");
    arm_set_home(1000);
    */
    char cmd[128];

    while (1) {
        // 接收串口数据
        if (uart_command_is_ready()) {
            uart_command_get_frame(cmd, sizeof(cmd));
            bsp_uart1_send_string("[RX] ");
            bsp_uart1_send_string(cmd);
            bsp_uart1_send_string("\r\n");

            bsp_uart3_send_string(cmd);
        }
    }
}
