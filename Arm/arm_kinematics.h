#ifndef __ARM_KINEMATICS_H__
#define __ARM_KINEMATICS_H__

#include <stdint.h>

/*
 * L0：地面 / 底座安装平面 → joint1 大臂转轴中心
 * L1：joint1 转轴中心 → joint2 转轴中心
 * L2：joint2 转轴中心 → joint3 腕部折叠转轴中心
 * L3：joint3 转轴中心 → 夹爪末端中心
*/

#define ARM_L0_MM 100
#define ARM_L1_MM 105
#define ARM_L2_MM 75
#define ARM_L3_MM 180

typedef struct {
    float x;
    float z;
} arm_point_2d_t;

/*
 * theta1 管大臂整体抬/倒
 * theta2 管小臂相对大臂折多少
 * theta3 管腕部相对小臂折多少
 */
arm_point_2d_t arm_forward_kinematics_2d(int16_t theta1, int16_t theta2,
                                         int16_t theta3);

#endif  // __ARM_KINEMATICS_H__
