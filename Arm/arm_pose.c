#include "arm_pose.h"

#include <stddef.h>

#include "arm_joint.h"

void arm_set_pose(const arm_pose_t* pose, uint16_t time) {
    if (pose == NULL) return;
    bus_servo_cmd_t cmds[SERVO_COUNT];
    // 转换成舵机位置
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        cmds[i].id = i;
        // 位置限幅
        cmds[i].pos = arm_joint_clamp_pwm(i, pose->joint[i]);
    }
    // 调用动作
    bus_servo_move_group(cmds, SERVO_COUNT, time);
}
void arm_set_home(uint16_t time) {
    arm_pose_t home;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        home.joint[i] = arm_joint_get_home(i);
    }
    arm_set_pose(&home, time);
}

void arm_play_frame(const arm_frame_t* frame) {
    if (frame == NULL) return;
    // 执行姿态
    arm_set_pose(&frame->pose, frame->time);
}

void arm_set_joint_angle(uint8_t joint_id, int16_t angle_deg, uint16_t time) {
    if (joint_id >= SERVO_COUNT) return;
    arm_pose_t pose;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose.joint[i] = arm_joint_get_home(i);
    }
    // 计算 角度 -> PWM 转换
    pose.joint[joint_id] = arm_joint_angle_to_pwm(joint_id, angle_deg);
    // 执行姿态
    arm_set_pose(&pose, time);
}

void arm_set_angle_pose(const int16_t angles[SERVO_COUNT], uint16_t time) {
    if (angles == NULL) return;
    arm_pose_t pose;
    // 解析角度
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pose.joint[i] = arm_joint_angle_to_pwm(i, angles[i]);
    }
    // 执行姿态
    arm_set_pose(&pose, time);
}
