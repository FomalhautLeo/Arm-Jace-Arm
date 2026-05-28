#include "bus_servo.h"

#include <stdio.h>
#include <string.h>

#include "bsp_uart.h"
#include "user_debug.h"
#include "user_utils.h"

static void bus_servo_send_cmd(const char* cmd) {
    bsp_uart3_send_string(cmd);
    debug_println(cmd);
}

void bus_servo_move(uint8_t id, uint16_t pos, uint16_t time) {
    // 先限制参数
    pos = CLAMP(pos, SERVO_POS_MIN, SERVO_POS_MAX);
    time = CLAMP(time, SERVO_TIME_MIN, SERVO_TIME_MAX);
    // 格式化命令，如："#000P1300T1000!"
    char cmd[SERVO_CMD_LENGTH];
    snprintf(cmd, SERVO_CMD_LENGTH, "#%03uP%04uT%04u!", id, pos, time);
    // 发送命令
    bus_servo_send_cmd(cmd);
}

void bus_servo_move_group(const bus_servo_cmd_t* cmds, uint8_t count,
                          uint16_t time) {
    if (cmds == NULL || count == 0) return;
    // 命令格式：{G0000#000P1602T1000!#001P2500T0000!#002P1500T1000!}
    char cmd[SERVO_GROUP_CMD_LENGTH];
    // 起始位置
    uint8_t idx = 0;
    int len = snprintf(cmd + idx, sizeof(cmd) - idx, SERVO_GROUP_PREFIX);
    if (len < 0 || len >= (sizeof(cmd) - idx)) return;
    idx += len;
    // 限制舵机数量
    count = MIN(count, SERVO_COUNT);
    // 限制时间
    time = CLAMP(time, SERVO_TIME_MIN, SERVO_TIME_MAX);
    for (uint8_t i = 0; i < count; i++) {
        len = snprintf(cmd + idx, sizeof(cmd) - idx, "#%03uP%04uT%04u!",
                       cmds[i].id,
                       CLAMP(cmds[i].pos, SERVO_POS_MIN, SERVO_POS_MAX), time);
        if (len < 0 || len >= (sizeof(cmd) - idx)) return;
        idx += len;
    }
    len = snprintf(cmd + idx, sizeof(cmd) - idx, "}");
    if (len < 0 || len >= sizeof(cmd) - idx) {
        return;
    }
    // 发送命令
    bus_servo_send_cmd(cmd);
}
