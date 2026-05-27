#ifndef __ARM_CALIB_H__
#define __ARM_CALIB_H__

#include <stdint.h>

// 限位标定，只动单个舵机
void arm_calib_move_joint(uint8_t joint_id, uint16_t pos, uint16_t time);

void arm_calib_home(uint16_t time);

#endif  // __ARM_CALIB_H__
