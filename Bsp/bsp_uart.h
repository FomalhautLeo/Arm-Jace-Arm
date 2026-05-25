#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include <stdint.h>

void bsp_uart1_init(uint32_t baudrate);

void bsp_uart1_send_byte(uint8_t data);

void bsp_uart1_send_string(const char* str);

#endif  // __BSP_UART_H__
