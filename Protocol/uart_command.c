#include "uart_command.h"

#include <string.h>

static volatile uint8_t frame_ready = 0;  // 标记是否就绪
static volatile uint8_t receiving = 0;    // 标记是否正在解析
static char rx_buf[128];                  // 命令缓冲区
static volatile uint16_t rx_index = 0;    // 命令缓冲区下标

void uart_command_on_byte(uint8_t data) {
    // 旧数据还没处理
    if (frame_ready) return;
    if ((char)data == '#') {
        // 获取到新数据，开始解析
        receiving = 1;
        rx_index = 0;
    }
    if (receiving) {
        // 放入缓冲区
        rx_buf[rx_index++] = data;
        if (rx_index >= sizeof(rx_buf) - 1) {
            // 越界，数据丢弃
            rx_index = 0;
            receiving = 0;
            frame_ready = 0;
            memset(rx_buf, 0, sizeof(rx_buf));
            return;
        }
        if ((char)data == '!') {
            // 命令结束
            frame_ready = 1;
            receiving = 0;
            rx_buf[rx_index] = '\0';
        }
    }
}

uint8_t uart_command_is_ready(void) { return frame_ready; }

void uart_command_get_frame(char* buf, uint16_t buf_size) {
    // 拷贝缓冲区
    strncpy(buf, rx_buf, buf_size - 1);
    buf[buf_size - 1] = '\0';
    // 缓冲区清空，状态重置
    memset(rx_buf, 0, sizeof(rx_buf));
    rx_index = 0;
    frame_ready = 0;
}
