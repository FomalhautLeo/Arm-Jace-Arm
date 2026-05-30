#include "arm_joint.h"

#include "bus_servo.h"
#include "user_utils.h"

static const arm_joint_config_t joint_configs[SERVO_COUNT] = {
    {550, 2450, 1500, +1},   // joint 0 底座
    {750, 2150, 1500, -1},   // joint 1 大臂
    {900, 2250, 1500, +1},   // joint 2 小臂
    {800, 2150, 1500, -1},   // joint 3 腕部折叠
    {550, 2450, 1500, +1},   // joint 4 腕部旋转
    {1100, 1690, 1500, +1},  // joint 5 夹爪
};

uint16_t arm_joint_clamp_pwm(uint8_t joint_id, uint16_t pwm) {
    if (joint_id >= SERVO_COUNT) {
        return SERVO_POS_MID;
    }
    return CLAMP(pwm, joint_configs[joint_id].min, joint_configs[joint_id].max);
}
uint16_t arm_joint_get_home(uint8_t joint_id) {
    if (joint_id >= SERVO_COUNT) {
        return SERVO_POS_MID;
    }
    return joint_configs[joint_id].home;
}

uint16_t arm_joint_angle_to_pwm(uint8_t joint_id, int16_t angle_deg) {
    if (joint_id >= SERVO_COUNT) {
        return SERVO_POS_MID;
    }
    // 取出限位配置
    const arm_joint_config_t* cfg = &joint_configs[joint_id];
    // 500~2500 对应 270°
    // 2000 / 270 = 200 / 27 ≈ 7.4074 PWM/°
    // 因此变化的 PWM 值为 deg * 2000 / 270
    // 用 int32 防溢出
    int32_t delta = (int32_t)angle_deg * 2000 / 270;
    // 按方向计算 PWM 变化
    int32_t pwm = cfg->home + cfg->dir * delta;
    // 限位
    pwm = CLAMP(pwm, cfg->min, cfg->max);
    return (uint16_t)pwm;
}
