#include "command_parser.h"

#include <stddef.h>
#include <stdint.h>

#include "arm_calib.h"
#include "arm_pose.h"
#include "bsp_uart.h"
#include "user_debug.h"
#include "user_def.h"

static void handle_joint(const char* cmd);
static void handle_pose(const char* cmd);
static void handle_home(const char* cmd);
static uint8_t parse_num(const char* str, uint16_t length, char end,
                         uint16_t* num, uint16_t* consumed);

void command_parser_handle(const char* cmd) {
    if (cmd == NULL) return;
    debug_print("[Parser] Parsing command: ");
    debug_println(cmd);
    if (cmd[0] == '#') {
        // 裸命令透传
        bsp_uart3_send_string(cmd);
        return;
    }
    // 不是合法命令
    if (cmd[0] != '$' || cmd[2] != ':') {
        debug_err("[Parser] invalid command");
        return;
    }
    // 区分命令类型
    switch (cmd[1]) {
        case 'J':
            // 单个关节运动
            handle_joint(cmd);
            break;
        case 'P':
            // 执行姿态动作
            handle_pose(cmd);
            break;
        case 'H':
            // 全部复原
            handle_home(cmd);
            break;
        default:
            break;
    }
}

// $J:joint,pos,time!  单个关节运动
static void handle_joint(const char* cmd) {
    uint16_t used;
    uint16_t idx = 3;
    // 需要解析的参数
    uint16_t joint;
    uint16_t pos;
    uint16_t time;
    // 1. 解析 joint
    if (!parse_num(cmd + idx, UART_COMMAND_BUFFER_LENGTH - idx, ',', &joint,
                   &used)) {
        debug_err("[$J] invalid joint");
        return;
    }
    if (joint >= SERVO_COUNT) {
        debug_err("[$J] joint out of range");
        return;
    }
    // 2. 解析 pos
    idx += used;
    if (!parse_num(cmd + idx, UART_COMMAND_BUFFER_LENGTH - idx, ',', &pos,
                   &used)) {
        debug_err("[$J] invalid pos");
        return;
    }
    // 3. 解析 time
    idx += used;
    if (!parse_num(cmd + idx, UART_COMMAND_BUFFER_LENGTH - idx, '!', &time,
                   &used)) {
        debug_err("[$J] invalid time");
        return;
    }
    // 4. 解析完成，执行动作
    arm_calib_move_joint((uint8_t)joint, pos, time);
    debug_ok("J");
}

// $P:p0,p1,p2,p3,p4,p5,time!  姿态动作
static void handle_pose(const char* cmd) {
    uint16_t used;
    uint16_t idx = 3;
    // 参数
    arm_pose_t pose;
    uint16_t time;
    // 逐个关节解析
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (!parse_num(cmd + idx, UART_COMMAND_BUFFER_LENGTH - idx, ',',
                       &(pose.joint[i]), &used)) {
            debug_err("[P] invalid joint pose");
            return;
        }
        idx += used;
    }
    // 解析时间
    if (!parse_num(cmd + idx, UART_COMMAND_BUFFER_LENGTH - idx, '!', &time,
                   &used)) {
        debug_err("[P] invalid time");
        return;
    }
    arm_set_pose(&pose, time);
    debug_ok("P");
}

// $H:time!  关节全部复原
static void handle_home(const char* cmd) {
    uint16_t used;
    uint16_t time;
    // 解析 time
    if (!parse_num(cmd + 3, UART_COMMAND_BUFFER_LENGTH - 3, '!', &time,
                   &used)) {
        debug_err("[H] invalid time");
        return;
    }
    arm_calib_home(time);
    debug_ok("H");
}

/*
 * 从字符串起始位置解析一个无符号整数，直到遇到指定结束符 end。
 *
 * 示例：
 *   输入 str = "1600,800!", end = ','
 *   解析结果：
 *     *num = 1600
 *     *consumed = 5   // "1600," 一共占 5 个字符，包含结束符 ','
 *
 * 参数：
 *   str       : 待解析字符串的起始地址
 *   length    : 最多允许读取的字符数，防止越界
 *   end       : 数字结束符，例如 ',' 或 '!'
 *   num       : 输出解析得到的数字
 *   consumed  : 输出本次解析消耗的字符数，包含结束符 end
 *
 * 返回：
 *   1：解析成功
 *   0：解析失败
 *
 * 失败情况：
 *   - 参数为空
 *   - 没有解析到任何数字
 *   - 数字中出现非数字字符
 *   - 在 length 范围内没有遇到 end
 */
static uint8_t parse_num(const char* str, uint16_t length, char end,
                         uint16_t* num, uint16_t* consumed) {
    uint16_t idx = 0;
    uint16_t res = 0;
    if (str == NULL || num == NULL || consumed == NULL) {
        return 0;
    }
    while (idx < length && str[idx] != end) {
        char ch = str[idx];
        // 无效字符，解析失败
        if (ch < '0' || ch > '9') return 0;
        // 十进制累加：例如 '1' '6' '0' '0' -> 1600
        res = res * 10 + (uint16_t)(ch - '0');
        idx++;
    }
    // 没有有效信息，解析失败
    if (idx == 0) return 0;
    // 未正常结束，解析失败
    if (idx >= length || str[idx] != end) return 0;
    *num = res;
    *consumed = idx + 1;  // 包含结束符
    return 1;
}
