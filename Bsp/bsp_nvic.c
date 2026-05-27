#include "bsp_nvic.h"

#include "misc.h"

void bsp_nvic_init() {
    // 全局中断配置
    // 抢占优先级和子优先级各占2位
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
