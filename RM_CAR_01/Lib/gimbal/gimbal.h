#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stm32f1xx_hal.h"

// 云台舵机引脚定义
#define GIMBAL_PITCH_PIN GPIO_PIN_0    // PA0 - 俯仰舵机 (上下)
#define GIMBAL_PITCH_PORT GPIOA
#define GIMBAL_YAW_PIN GPIO_PIN_1      // PA1 - 偏航舵机 (左右)

// 水弹发射电机引脚
#define WATER_BULLET_PIN GPIO_PIN_5    // PA2 - 水弹发射电机控制
#define WATER_BULLET_PORT GPIOB

// 云台角度限制
#define GIMBAL_PITCH_MIN 30    // 俯仰最小角度
#define GIMBAL_PITCH_MAX 150   // 俯仰最大角度
#define GIMBAL_YAW_MIN 2       // 偏航最小角度
#define GIMBAL_YAW_MAX 180     // 偏航最大角度

// 云台控制结构体
typedef struct {
    uint8_t current_pitch;     // 当前俯仰角度
    uint8_t current_yaw;       // 当前偏航角度
    uint8_t target_pitch;      // 目标俯仰角度
    uint8_t target_yaw;        // 目标偏航角度
    TIM_HandleTypeDef* pwm_tim; // PWM定时器
} Gimbal_TypeDef;

// 水弹发射控制
typedef struct {
    uint8_t is_shooting;       // 是否正在发射
} WaterBullet_TypeDef;

// 函数声明
void Gimbal_Init(Gimbal_TypeDef* gimbal, TIM_HandleTypeDef* pwm_tim);
void Gimbal_SetAngle(Gimbal_TypeDef* gimbal, uint8_t pitch, uint8_t yaw);
void Gimbal_ControlWithPS2(Gimbal_TypeDef* gimbal, uint8_t RJoy_UD, uint8_t RJoy_LR, uint8_t btn1, uint8_t btn2);
void Gimbal_Update(Gimbal_TypeDef* gimbal);
void WaterBullet_Init(WaterBullet_TypeDef* shooter);
void WaterBullet_ControlWithPS2(WaterBullet_TypeDef* shooter, uint8_t btn2);

#endif