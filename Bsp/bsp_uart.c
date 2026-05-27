#include "bsp_uart.h"

#include "stm32f10x_rcc.h"

void bsp_uart1_init(uint32_t baudrate) {
    // 开启 GPIOA + USART1 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,
                           ENABLE);
    // USART1 重置
    USART_DeInit(USART1);

    // GPIO 引脚初始化
    // PA9 TXD1 发送端，推挽复用
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = GPIO_Pin_9;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
    // PA10 RXD1 接收端，输入悬空
    gpio_init.GPIO_Pin = GPIO_Pin_10;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_init);

    // USART1 初始化
    USART_InitTypeDef usart_init;
    usart_init.USART_BaudRate = baudrate;               // 波特率
    usart_init.USART_WordLength = USART_WordLength_8b;  // 一次传8位/一字节
    usart_init.USART_StopBits = USART_StopBits_1;       // 1停止位
    usart_init.USART_Parity = USART_Parity_No;          // 不校验
    usart_init.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;                     // 不流控
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // 收发模式
    USART_Init(USART1, &usart_init);

    // 使能
    USART_Cmd(USART1, ENABLE);
}

void bsp_uart3_init(uint32_t baudrate) {
    // 开启 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // 开启 USART3 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // USART3 重置
    USART_DeInit(USART3);

    // GPIO 引脚初始化
    // PB10 TXD3 发送端，推挽复用
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = GPIO_Pin_10;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_init);
    // PB11 RXD3 接收端，输入悬空
    gpio_init.GPIO_Pin = GPIO_Pin_11;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio_init);

    // USART3 初始化
    USART_InitTypeDef usart_init;
    usart_init.USART_BaudRate = baudrate;               // 波特率
    usart_init.USART_WordLength = USART_WordLength_8b;  // 一次传8位/一字节
    usart_init.USART_StopBits = USART_StopBits_1;       // 1停止位
    usart_init.USART_Parity = USART_Parity_No;          // 不校验
    usart_init.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;                     // 不流控
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // 收发模式
    USART_Init(USART3, &usart_init);

    // 使能
    USART_Cmd(USART3, ENABLE);
}

void bsp_uart_send_byte(USART_TypeDef* usart_x, uint8_t data) {
    // 发送数据
    USART_SendData(usart_x, data);
    // 等待寄存器空，可以继续写入
    while (USART_GetFlagStatus(usart_x, USART_FLAG_TXE) != SET) {
    }
}

void bsp_uart_send_string(USART_TypeDef* usart_x, const char* str) {
    // 遍历字符串发送
    while (*str) {
        bsp_uart_send_byte(usart_x, (uint8_t)*str);
        str++;
    }
}

void bsp_uart1_send_string(const char* str) {
    bsp_uart_send_string(USART1, str);
}

void bsp_uart3_send_string(const char* str) {
    bsp_uart_send_string(USART3, str);
}
