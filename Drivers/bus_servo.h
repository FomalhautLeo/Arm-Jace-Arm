#ifndef __BUS_SERVO_H__
#define __BUS_SERVO_H__

#include <stdint.h>

#define SERVO_COUNT 6  // 一共6个舵机

#define SERVO_CMD_LENGTH 32
#define SERVO_GROUP_CMD_LENGTH 128

// 动作组前缀
#define SERVO_GROUP_PREFIX "{G0001"

// 参数限制
#define SERVO_POS_MIN 500
#define SERVO_POS_MAX 2500
#define SERVO_TIME_MIN 500
#define SERVO_TIME_MAX 9999

typedef struct {
    uint8_t id;
    uint16_t pos;
} bus_servo_cmd_t;

// 单个舵机运动
void bus_servo_move(uint8_t id, uint16_t pos, uint16_t time);
// 多个舵机同步运动
void bus_servo_move_group(const bus_servo_cmd_t* cmds, uint8_t count,
                          uint16_t time);

#endif  // __BUS_SERVO_H__
