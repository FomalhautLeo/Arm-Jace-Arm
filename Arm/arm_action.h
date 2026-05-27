#ifndef __ARM_ACTION_H__
#define __ARM_ACTION_H__

#include "arm_pose.h"

// 动作组
typedef struct {
    const arm_frame_t* frames;
    uint16_t frame_count;
} arm_action_t;

// 执行动作组
void arm_play_action(const arm_action_t* action);

#endif  // __ARM_ACTION_H__
