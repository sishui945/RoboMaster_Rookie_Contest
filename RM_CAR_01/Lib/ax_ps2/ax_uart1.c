#include "ax_uart1.h"
#include <stdio.h>

#include "usart.h"

extern UART_HandleTypeDef huart1;  // 由CubeMX生成

void AX_UART1_Init(uint32_t baud)
{
    // UART配置在CubeMX中完成
    // 这个函数可以留空，或者用于自定义配置
}

// 重定向printf
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int __io_getchar(void)
{
    uint8_t ch;
    HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
    return ch;
}