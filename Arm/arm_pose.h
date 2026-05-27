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

// 姿态限幅
typedef struct {
    uint16_t min;
    uint16_t max;
    uint16_t home;
} arm_joint_limit_t;

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

#endif  // __ARM_POSE_H__
