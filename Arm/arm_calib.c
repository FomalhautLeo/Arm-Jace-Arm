#include "arm_calib.h"

#include "arm_pose.h"

void arm_calib_move_joint(uint8_t joint_id, uint16_t pos, uint16_t time) {
    arm_pose_t pose = {.joint = {1500, 1500, 1500, 1500, 1500, 1500}};

    if (joint_id >= SERVO_COUNT) return;

    pose.joint[joint_id] = pos;
    arm_set_pose(&pose, time);
}

void arm_calib_home(uint16_t time) {
    arm_set_home(time);
}
