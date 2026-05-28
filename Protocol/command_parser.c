#include "command_parser.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "arm_calib.h"
#include "arm_pose.h"
#include "bsp_uart.h"
#include "user_debug.h"
#include "user_def.h"

typedef struct {
    const char* str;
    uint16_t idx;
} command_reader_t;

static void command_reader_init(command_reader_t* reader, const char* cmd);
static uint8_t command_read_i16(command_reader_t* reader, char end,
                                int16_t* value);
static uint8_t command_read_u16(command_reader_t* reader, char end,
                                uint16_t* value);

static void handle_joint(command_reader_t* reader);
static void handle_pose(command_reader_t* reader);
static void handle_home(command_reader_t* reader);
static void handle_angle(command_reader_t* reader);
static void handle_angle_pose(command_reader_t* reader);

// 解析字符串中的数字
static uint8_t parse_num(const char* str, uint16_t length, char end,
                         int16_t* num, uint16_t* consumed);

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
    // 先初始化 reader
    command_reader_t reader;
    command_reader_init(&reader, cmd);
    // 区分命令类型
    switch (cmd[1]) {
        case 'J':
            // 单个关节运动
            handle_joint(&reader);
            break;
        case 'P':
            // 执行姿态动作
            handle_pose(&reader);
            break;
        case 'H':
            // 全部复原
            handle_home(&reader);
            break;
        case 'A':
            handle_angle(&reader);
            break;
        case 'Q':
            handle_angle_pose(&reader);
            break;
        default:
            debug_err("[Parser] unknown command");
            break;
    }
}

static void command_reader_init(command_reader_t* reader, const char* cmd) {
    if (reader == NULL || cmd == NULL) return;
    const uint16_t kCmdPrefixLength = 3;  // ${Cmd}: 共3字节
    reader->str = cmd;
    reader->idx = kCmdPrefixLength;
}

static uint8_t command_read_i16(command_reader_t* reader, char end,
                                int16_t* value) {
    if (reader == NULL || reader->str == NULL || value == NULL) return 0;
    if (reader->idx >= UART_COMMAND_BUFFER_LENGTH) return 0;
    uint16_t used;
    int16_t num;
    // 解析数字
    if (!parse_num(reader->str + reader->idx,
                   UART_COMMAND_BUFFER_LENGTH - reader->idx, end, &num,
                   &used)) {
        return 0;
    }
    reader->idx += used;
    *value = num;
    return 1;
}
static uint8_t command_read_u16(command_reader_t* reader, char end,
                                uint16_t* value) {
    if (reader == NULL || reader->str == NULL || value == NULL) return 0;
    if (reader->idx >= UART_COMMAND_BUFFER_LENGTH) return 0;
    uint16_t used;
    int16_t num;
    // 解析数字
    if (!parse_num(reader->str + reader->idx,
                   UART_COMMAND_BUFFER_LENGTH - reader->idx, end, &num,
                   &used)) {
        return 0;
    }
    if (num < 0) {
        return 0;
    }
    reader->idx += used;
    *value = (uint16_t)num;
    return 1;
}

// $J:joint,pos,time!  单个关节运动
static void handle_joint(command_reader_t* reader) {
    uint16_t joint;
    uint16_t pos;
    uint16_t time;
    // 1. 解析 joint
    if (!command_read_u16(reader, ',', &joint)) {
        debug_err("[$J] invalid joint");
        return;
    }
    if (joint >= SERVO_COUNT) {
        debug_err("[$J] joint out of range");
        return;
    }
    // 2. 解析 pos
    if (!command_read_u16(reader, ',', &pos)) {
        debug_err("[$J] invalid pos");
        return;
    }
    // 3. 解析 time
    if (!command_read_u16(reader, '!', &time)) {
        debug_err("[$J] invalid time");
        return;
    }
    // 4. 解析完成，执行动作
    arm_calib_move_joint((uint8_t)joint, (uint16_t)pos, (uint16_t)time);
    debug_ok("J");
}

// $P:p0,p1,p2,p3,p4,p5,time!  姿态动作
static void handle_pose(command_reader_t* reader) {
    arm_pose_t pose;
    uint16_t time;
    uint16_t pwm;
    // 逐个关节解析
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (!command_read_u16(reader, ',', &pwm)) {
            debug_err("[P] invalid joint pose");
            return;
        }
        pose.joint[i] = pwm;
    }
    // 解析时间
    if (!command_read_u16(reader, '!', &time)) {
        debug_err("[P] invalid time");
        return;
    }
    arm_set_pose(&pose, (uint16_t)time);
    debug_ok("P");
}

// $H:time!  关节全部复原
static void handle_home(command_reader_t* reader) {
    uint16_t time;
    // 解析 time
    if (!command_read_u16(reader, '!', &time)) {
        debug_err("[H] invalid time");
        return;
    }
    arm_calib_home((uint16_t)time);
    debug_ok("H");
}

// $A:joint,angle,time!  单个关节按角度运动
static void handle_angle(command_reader_t* reader) {
    uint16_t joint;
    int16_t angle;
    uint16_t time;
    // 1. 解析 joint
    if (!command_read_u16(reader, ',', &joint)) {
        debug_err("[$A] invalid joint");
        return;
    }
    if (joint >= SERVO_COUNT) {
        debug_err("[$A] joint out of range");
        return;
    }
    // 2. 解析 angle
    if (!command_read_i16(reader, ',', &angle)) {
        debug_err("[$A] invalid angle");
        return;
    }
    // 3. 解析 time
    if (!command_read_u16(reader, '!', &time)) {
        debug_err("[$A] invalid time");
        return;
    }
    // 4. 解析完成，执行动作
    arm_set_joint_angle((uint8_t)joint, angle, (uint16_t)time);
    debug_ok("A");
}

// $Q:a0,a1,a2,a3,a4,a5,time!  指定角度姿态动作
static void handle_angle_pose(command_reader_t* reader) {
    uint16_t time;
    int16_t angles[SERVO_COUNT];
    // 逐个关节解析
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (!command_read_i16(reader, ',', &angles[i])) {
            debug_err("[Q] invalid joint angle pose");
            return;
        }
    }
    // 解析时间
    if (!command_read_u16(reader, '!', &time)) {
        debug_err("[Q] invalid time");
        return;
    }
    arm_set_angle_pose(angles, time);
    debug_ok("Q");
}

/*
 * 从字符串起始位置解析一个有符号整数，直到遇到指定结束符 end。
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
                         int16_t* num, uint16_t* consumed) {
    uint16_t idx = 0;
    int32_t res = 0;
    uint8_t has_digit = 0;
    if (str == NULL || num == NULL || consumed == NULL || length == 0) {
        return 0;
    }
    int8_t sign = 1;
    if (str[0] == '-') {
        sign = -1;
        idx++;
    }
    while (idx < length && str[idx] != end) {
        char ch = str[idx];
        // 无效字符，解析失败
        if (ch < '0' || ch > '9') return 0;
        // 十进制累加：例如 '1' '6' '0' '0' -> 1600
        res = res * 10 + (int32_t)(ch - '0');
        has_digit = 1;
        idx++;
    }
    // 没有有效信息，解析失败
    if (has_digit == 0) return 0;
    // 未正常结束，解析失败
    if (idx >= length || str[idx] != end) return 0;

    res *= sign;
    // 范围超限
    if (res < INT16_MIN || res > INT16_MAX) return 0;

    *num = (int16_t)res;
    *consumed = idx + 1;  // 包含结束符
    return 1;
}
