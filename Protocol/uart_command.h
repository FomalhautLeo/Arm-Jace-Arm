#ifndef __UART_COMMAND_H__
#define __UART_COMMAND_H__

#include <stdint.h>

// 写入字节数据
void uart_command_on_byte(uint8_t data);
// 串口命令是否解析完成
uint8_t uart_command_is_ready(void);
// 获取完整串口命令
void uart_command_get_frame(char* buf, uint16_t buf_size);

#endif  // __UART_COMMAND_H__
