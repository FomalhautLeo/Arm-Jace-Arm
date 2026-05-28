#ifndef __ARM_POSE_H__
#define __ARM_POSE_H__

/*
姿态控制
*/

#include "bus_servo.h"

// 姿态
typedef struct {
    uint16_t joint[SERVO_COUNT];
} arm_pose_t;

// 动作帧
typedef struct {
    arm_pose_t pose;
    uint16_t time;
} arm_frame_t;

// 设置姿态
void arm_set_pose(const arm_pose_t* pose, uint16_t time);

// 姿态回正
void arm_set_home(uint16_t time);

// 动作帧执行
void arm_play_frame(const arm_frame_t* frame);

// 单关节角度姿态
void arm_set_joint_angle(uint8_t joint_id, int16_t angle_deg, uint16_t time);

// 角度姿态执行
void arm_set_angle_pose(const int16_t angles[SERVO_COUNT], uint16_t time);

#endif  // __ARM_POSE_H__
