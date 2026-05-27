#include "arm_action.h"

#include <stddef.h>

#include "bsp_systick.h"

// 帧间停顿100ms
static const uint32_t kFrameBufferTimeMs = 100;

void arm_play_action(const arm_action_t* action) {
    if (action == NULL) return;
    if (action->frames == NULL) return;
    if (action->frame_count == 0) return;
    // 逐个执行动作
    for (uint16_t i = 0; i < action->frame_count; i++) {
        const arm_frame_t* frame = action->frames + i;
        arm_play_frame(frame);
        // 等待动作执行完成
        bsp_delay_ms((uint32_t)frame->time + kFrameBufferTimeMs);
    }
}
