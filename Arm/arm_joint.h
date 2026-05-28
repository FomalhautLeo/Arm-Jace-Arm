#ifndef __ARM_JOINT_H__
#define __ARM_JOINT_H__

#include <stdint.h>

// 姿态限幅
typedef struct {
    uint16_t min;
    uint16_t max;
    uint16_t home;
    // dir = +1：角度增大 → PWM 增大
    // dir = -1：角度增大 → PWM 减小
    int8_t dir;
} arm_joint_config_t;

// 关节姿态限位
uint16_t arm_joint_clamp_pwm(uint8_t joint_id, uint16_t pwm);
// 关节初始姿态
uint16_t arm_joint_get_home(uint8_t joint_id);
// 角度转换为 PWM 姿态值
uint16_t arm_joint_angle_to_pwm(uint8_t joint_id, int16_t angle_deg);

#endif  // __ARM_JOINT_H__
