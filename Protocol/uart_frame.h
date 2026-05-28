#ifndef __UART_FRAME_H__
#define __UART_FRAME_H__

#include <stdint.h>

#include "user_def.h"

// 串口数据来源
typedef enum {
    UART_FRAME_USER = 0,   // USART1：用户命令
    UART_FRAME_SERVO = 1,  // USART3：舵机返回
    UART_FRAME_COUNT = 2,
} uart_frame_channel_t;

typedef struct {
    volatile uint8_t ready;                   // 是否就绪
    volatile uint8_t receiving;               // 是否正在解析
    volatile uint16_t index;                  // 帧缓冲区下标
    char buffer[UART_COMMAND_BUFFER_LENGTH];  // 帧缓冲区
} uart_frame_t;

// 写入字节数据
void uart_frame_on_byte(uart_frame_channel_t channel, uint8_t data);
// 串口帧是否解析完成
uint8_t uart_frame_is_ready(uart_frame_channel_t channel);
// 获取完整串口帧
void uart_frame_get(uart_frame_channel_t channel, char* buf, uint16_t buf_size);

#endif  // __UART_FRAME_H__
