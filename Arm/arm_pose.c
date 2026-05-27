#include "arm_pose.h"

#include <stddef.h>

#include "user_utils.h"

static const arm_joint_limit_t joint_limits[SERVO_COUNT] = {
    {SERVO_POS_MIN, SERVO_POS_MAX, SERVO_POS_MID},
    {SERVO_POS_MIN, SERVO_POS_MAX, SERVO_POS_MID},
    {SERVO_POS_MIN, SERVO_POS_MAX, SERVO_POS_MID},
    {SERVO_POS_MIN, SERVO_POS_MAX, SERVO_POS_MID},
    {SERVO_POS_MIN, SERVO_POS_MAX, SERVO_POS_MID},
    {SERVO_POS_MIN, SERVO_POS_MAX, SERVO_POS_MID},
};

void arm_set_pose(const arm_pose_t* pose, uint16_t time) {
    if (pose == NULL) return;
    bus_servo_cmd_t cmds[SERVO_COUNT];
    // 转换成舵机位置
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        cmds[i].id = i;
        // 位置限幅
        cmds[i].pos =
            CLAMP(pose->joint[i], joint_limits[i].min, joint_limits[i].max);
    }
    // 调用动作
    bus_servo_move_group(cmds, SERVO_COUNT, time);
}
void arm_set_home(uint16_t time) {
    arm_pose_t home;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        home.joint[i] = joint_limits[i].home;
    }
    arm_set_pose(&home, time);
}

void arm_play_frame(const arm_frame_t* frame) {
    if (frame == NULL) return;
    // 执行姿态
    arm_set_pose(&frame->pose, frame->time);
}
