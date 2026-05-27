#include "user_debug.h"

#include "bsp_uart.h"

void debug_print(const char* str) {
#ifdef BUS_SERVO_DEBUG
    bsp_uart1_send_string(str);
#endif
}

void debug_println(const char* str) {
#ifdef BUS_SERVO_DEBUG
    bsp_uart1_send_string(str);
    bsp_uart1_send_string("\r\n");
#endif
}
