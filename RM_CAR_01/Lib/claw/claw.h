#ifndef __CLAW_H
#define __CLAW_H

#include "stm32f1xx_hal.h"

// 抓取机构舵机引脚定义
#define CLAW_LIFT_PIN GPIO_PIN_10      // PB10 - 升降舵机 (MG996R 360度)
#define CLAW_LIFT_PORT GPIOB
#define CLAW_GRIP_PIN GPIO_PIN_11      // PB11 - 抓取舵机 (SG90 180度)
#define CLAW_GRIP_PORT GPIOB

// 抓取状态枚举
typedef enum {
    CLAW_STATE_OPEN = 0,       // 爪子张开
    CLAW_STATE_CLOSED = 1          // 爪子闭合
} ClawState_TypeDef;

// 抓取控制结构体
typedef struct {
    ClawState_TypeDef grip_state;      // 抓取状态
    uint8_t is_lifting;                // 是否正在升降
    TIM_HandleTypeDef* pwm_tim;        // PWM定时器
} Claw_TypeDef;

// 函数声明
void Claw_Init(Claw_TypeDef* claw, TIM_HandleTypeDef* pwm_tim);
void Claw_ControlWithPS2(Claw_TypeDef* claw, uint8_t btn2);
void Claw_Update(Claw_TypeDef* claw);

// MG996R 360度舵机控制函数
void Claw_Lift_Up(Claw_TypeDef* claw);
void Claw_Lift_Down(Claw_TypeDef* claw);
void Claw_Lift_Stop(Claw_TypeDef* claw);

// SG90 180度舵机角度控制
void Claw_Grip_Open(Claw_TypeDef* claw);
void Claw_Grip_Close(Claw_TypeDef* claw);

#endif