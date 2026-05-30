#include "arm_kinematics.h"

#include <math.h>

static const float kPi = 3.14159265358979323846;

const static float deg2rad(int16_t degree) { return degree * kPi / 180.0f; }

arm_point_2d_t arm_forward_kinematics_2d(int16_t theta1, int16_t theta2,
                                         int16_t theta3) {
    /*
     * 侧视平面坐标系：
     * x 正方向：机械臂向前伸出的方向
     * z 正方向：竖直向上
     *
     * phi 是每段连杆相对 x 正方向的绝对角度：
     *
     *   phi = pi/2 ：连杆竖直向上
     *   phi < pi/2 ：连杆向 x 正方向倾斜
     *   phi > pi/2 ：连杆向 x 负方向倾斜
     *
     * theta 是关节相对 home 的软件角度。
     * 先假设 home 姿态下三段连杆都近似竖直，
     * 即 home 时每段连杆的 phi 都是 pi/2。
     *
     * 因此：
     *
     *   phi1 = pi/2 - theta1
     *   phi2 = pi/2 - theta1 - theta2
     *   phi3 = pi/2 - theta1 - theta2 - theta3
     *
     * 正运动学：
     *
     *   x = L1 * cos(phi1)
     *     + L2 * cos(phi2)
     *     + L3 * cos(phi3)
     *
     *   z = L0
     *     + L1 * sin(phi1)
     *     + L2 * sin(phi2)
     *     + L3 * sin(phi3)
     *
     * 注意：
     *   公式计算时 theta / phi 使用弧度；
     *   外部输入的关节角度仍然使用角度制。
     */
    arm_point_2d_t res;

    float phi1 = kPi / 2.0f - deg2rad(theta1);
    float phi2 = kPi / 2.0f - deg2rad(theta1 + theta2);
    float phi3 = kPi / 2.0f - deg2rad(theta1 + theta2 + theta3);

    res.x = ARM_L1_MM * cosf(phi1) + ARM_L2_MM * cosf(phi2) +
            ARM_L3_MM * cosf(phi3);
    res.z = ARM_L0_MM + ARM_L1_MM * sinf(phi1) + ARM_L2_MM * sinf(phi2) +
            ARM_L3_MM * sinf(phi3);
    return res;
}

