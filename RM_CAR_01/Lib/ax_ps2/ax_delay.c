#include "ax_delay.h"
void AX_DELAY_Init(void)
{
	// HAL库会自动初始化SysTick，这里可以留空或添加自定义初始化
}

void AX_Delayus(uint16_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = (SystemCoreClock / 1000000) * us;

	while((DWT->CYCCNT - start) < cycles);
}

void AX_Delayms(uint16_t ms)
{
	HAL_Delay(ms);
}