#ifndef __AX_DELAY_H
#define __AX_DELAY_H

#include "stm32f1xx_hal.h"

void AX_DELAY_Init(void);
void AX_Delayus(uint16_t us);
void AX_Delayms(uint16_t ms);

#endif