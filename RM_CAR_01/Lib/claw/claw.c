#include "claw.h"
#include "main.h"
#include "tim.h"

// 角度转PWM比较值 (SG90 180度舵机)
static uint32_t AngleToCompare_SG90(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return (uint32_t)(500 + angle * 2000 / 180);
}
/**
  * @brief  控制收集装置的上下
  * @param  direction: 方向 0=正转, 1=反转
  * @param  speed: 速度范围0-49 (越大越慢)389 - 2581
  * @note   主时钟 72 MHz，PSC=71，ARR=19999 -> 计数频率 1 MHz，周期 20 ms
  */
void Steering_Engine_360(uint8_t direction, uint8_t speed) {
    uint16_t pulse_width;

    if (speed >= 50) speed = 49; // 限制速度范围

    if (direction == 0) {
        // 正转
        // 脉宽范围：500(最快) ~ 1500(停止)
        // 速度0(最快) → 脉宽500
        // 速度49(最慢) → 脉宽接近1500
        pulse_width = 1500 - speed * ((1500 - 389) / 49);
    } else {
        // 反转
        // 脉宽范围：1500(停止) ~ 2500(最快)
        // 速度0(最慢) → 脉宽1500    1.5-2.5
        // 速度49(最快) → 脉宽接近2500
        pulse_width = 1500 + speed * ((2581 - 1500) / 49);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_width);
}
// MG996R 360度舵机速度控制
static void MG996R_SetSpeed(TIM_HandleTypeDef* tim, uint32_t channel, int8_t speed)
{
    // 速度范围: -100 到 100
    // 对应PWM: 5% (1ms) 到 10% (2ms)
    if(speed < -100) speed = -100;
    if(speed > 100) speed = 100;
    
    // 映射到PWM值: 1000 (1ms) 到 2000 (2ms)
    uint32_t pwm_value = 1500 + speed * 5;  // 中位1500，±500
    __HAL_TIM_SET_COMPARE(tim, channel, pwm_value);
}

// 抓取机构初始化
void Claw_Init(Claw_TypeDef* claw, TIM_HandleTypeDef* pwm_tim)
{
    claw->pwm_tim = pwm_tim;
    claw->grip_state = CLAW_STATE_OPEN;
    claw->is_lifting = 0;

    // 初始化位置
    Claw_Grip_Open(claw);
    Claw_Lift_Stop(claw);
}

// 基于PS2控制抓取机构
void Claw_ControlWithPS2(Claw_TypeDef* claw, uint8_t btn2)
{
    static uint8_t last_btn2 = 0;

    // ○按钮 (0x40) - 切换抓取状态
    if((btn2 & 0x40) && !(last_btn2 & 0x40)) {
        if(claw->grip_state == CLAW_STATE_OPEN) {
            Claw_Grip_Close(claw);
            claw->grip_state = CLAW_STATE_CLOSED;
        } else {
            Claw_Grip_Open(claw);
            claw->grip_state = CLAW_STATE_OPEN;
        }
    }

    // L2按钮 (0x01) - 升起夹子
    if(btn2 & 0x01) {
        if(!claw->is_lifting) {
            claw->is_lifting = 1;
            Claw_Lift_Up(claw);
        }
    }
    // R2按钮 (0x02) - 降下夹子
    else if(btn2 & 0x02) {
        if(!claw->is_lifting) {
            claw->is_lifting = 1;
            Claw_Lift_Down(claw);
        }
    }
    // 没有按下L2或R2时停止升降
    else {
        if(claw->is_lifting) {
            claw->is_lifting = 0;
            Claw_Lift_Stop(claw);
        }
    }

    last_btn2 = btn2;
}

// 更新抓取机构状态
void Claw_Update(Claw_TypeDef* claw)
{
    // 这里可以添加其他状态更新逻辑
    // 目前只需要实时响应按钮，不需要额外更新
}

// MG996R 360度舵机控制函数
void Claw_Lift_Up(Claw_TypeDef* claw)
{
    // 正转 - 上升
    MG996R_SetSpeed(claw->pwm_tim, TIM_CHANNEL_3, 80);  // 使用TIM_CHANNEL_3
}

void Claw_Lift_Down(Claw_TypeDef* claw)
{
    // 反转 - 下降
    MG996R_SetSpeed(claw->pwm_tim, TIM_CHANNEL_3, -80); // 使用TIM_CHANNEL_3
}

void Claw_Lift_Stop(Claw_TypeDef* claw)
{
    // 停止
    MG996R_SetSpeed(claw->pwm_tim, TIM_CHANNEL_3, 0);
}

// SG90 180度舵机控制函数
void Claw_Grip_Open(Claw_TypeDef* claw)
{
    // 张开爪子 - 0度
    __HAL_TIM_SET_COMPARE(claw->pwm_tim, TIM_CHANNEL_4, AngleToCompare_SG90(0));
}

void Claw_Grip_Close(Claw_TypeDef* claw)
{
    // 闭合爪子 - 180度
    __HAL_TIM_SET_COMPARE(claw->pwm_tim, TIM_CHANNEL_4, AngleToCompare_SG90(180));
}