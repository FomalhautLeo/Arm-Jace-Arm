#include "uart_frame.h"

#include <string.h>

#include "user_def.h"

static uart_frame_t frame_arr[UART_FRAME_COUNT];

// 重置帧状态
static void reset_frame(uart_frame_t* frame) {
    if (frame == NULL) return;

    frame->index = 0;
    frame->receiving = 0;
    frame->ready = 0;
    memset(frame->buffer, 0, sizeof(frame->buffer));
}

void uart_frame_on_byte(uart_frame_channel_t channel, uint8_t data) {
    if (channel >= UART_FRAME_COUNT) return;
    uart_frame_t* frame = &frame_arr[channel];
    // 旧数据还没处理
    if (frame->ready) return;
    if ((char)data == '#' || (char)data == '$') {
        // 获取到新数据，开始解析
        frame->receiving = 1;
        frame->index = 0;
        memset(frame->buffer, 0, sizeof(frame->buffer));
    }
    if (frame->receiving) {
        // 放入缓冲区
        frame->buffer[frame->index++] = data;
        if (frame->index >= sizeof(frame->buffer) - 1) {
            // 越界，数据丢弃
            reset_frame(frame);
            return;
        }
        if ((char)data == '!') {
            // 帧结束
            frame->ready = 1;
            frame->receiving = 0;
            frame->buffer[frame->index] = '\0';
        }
    }
}

uint8_t uart_frame_is_ready(uart_frame_channel_t channel) {
    if (channel >= UART_FRAME_COUNT) return 0;
    return frame_arr[channel].ready;
}

void uart_frame_get(uart_frame_channel_t channel, char* buf,
                    uint16_t buf_size) {
    if (buf == NULL || buf_size == 0) return;
    if (channel >= UART_FRAME_COUNT) return;
    uart_frame_t* frame = &frame_arr[channel];
    // 拷贝缓冲区
    strncpy(buf, frame->buffer, buf_size - 1);
    buf[buf_size - 1] = '\0';
    // 缓冲区清空，状态重置
    reset_frame(frame);
}
