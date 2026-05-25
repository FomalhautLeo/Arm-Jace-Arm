#include "bsp_rcc.h"

#include "stm32f10x_flash.h"
#include "stm32f10x_rcc.h"

void bsp_rcc_init(void) {
    // RCC 恢复初始状态
    RCC_DeInit();
    // 开启外部时钟
    RCC_HSEConfig(RCC_HSE_ON);
    // 等待外部时钟准备好，若启动失败则主动卡死
    ErrorStatus hse_status;
    do {
        hse_status = RCC_WaitForHSEStartUp();
    } while (hse_status == ERROR);

    // 设置片内 Flash 等待和预取
    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    // 内部时钟设置
    // AHB 与 SYSCLK同频
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    // APB1 = AHB / 2
    RCC_PCLK1Config(RCC_HCLK_Div2);
    // APB2 = AHB / 1
    RCC_PCLK2Config(RCC_HCLK_Div1);

    // 设置倍频，8MHz -> 72MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    // 等待PLL完成
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET) {
    }

    // 设置 SYSCLK 源为PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    // 检查
    while (RCC_GetSYSCLKSource() != RCC_CFGR_SWS_PLL) {
    }
}
