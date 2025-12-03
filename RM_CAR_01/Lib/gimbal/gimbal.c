#include "gimbal.h"

#include <math.h>
#include <stdlib.h>

#include "main.h"

/**
  * @brief  把角度转换成 TIM 通道的比较值（CCR）
  * @param  angle: 0 ~ 180 度
  * @retval 比较值，范围 500 ~ 2500
  * @note   主时钟 72 MHz，PSC=71，ARR=19999 -> 计数频率 1 MHz，周期 20 ms
  */
static uint32_t AngleToCompare(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return (uint32_t)(500 + angle * 2000.0f / 180.0f);  // 使用浮点或更安全的计算
}

// 云台初始化
void Gimbal_Init(Gimbal_TypeDef* gimbal, TIM_HandleTypeDef* pwm_tim)
{
    gimbal->pwm_tim = pwm_tim;
    gimbal->current_pitch = 90;
    gimbal->current_yaw = 90;
    gimbal->target_pitch = 90;
    gimbal->target_yaw = 90;

    // 设置初始位置
    __HAL_TIM_SET_COMPARE(gimbal->pwm_tim, TIM_CHANNEL_1, AngleToCompare(gimbal->current_pitch));
    __HAL_TIM_SET_COMPARE(gimbal->pwm_tim, TIM_CHANNEL_2, AngleToCompare(gimbal->current_yaw));
}

// 设置云台角度
void Gimbal_SetAngle(Gimbal_TypeDef* gimbal, uint8_t pitch, uint8_t yaw)
{
    if(pitch < GIMBAL_PITCH_MIN) pitch = GIMBAL_PITCH_MIN;
    if(pitch > GIMBAL_PITCH_MAX) pitch = GIMBAL_PITCH_MAX;
    if(yaw < GIMBAL_YAW_MIN) yaw = GIMBAL_YAW_MIN;
    if(yaw > GIMBAL_YAW_MAX) yaw = GIMBAL_YAW_MAX;

    gimbal->target_pitch = pitch;
    gimbal->target_yaw = yaw;
}
/*
// 基于PS2手柄控制云台
void Gimbal_ControlWithPS2(Gimbal_TypeDef* gimbal, uint8_t RJoy_UD, uint8_t RJoy_LR, uint8_t btn1, uint8_t btn2)
{
    #define GIMBAL_DEAD_ZONE 20
    #define GIMBAL_SPEED 3
    // #define GIMBAL_SPEED 2

    int16_t ud_value = 128 - (int16_t)RJoy_UD;  // 上=正，下=负
    int16_t lr_value = (int16_t)RJoy_LR - 128;  // 右=正，左=负

    // 应用死区
    if(abs(ud_value) < GIMBAL_DEAD_ZONE) ud_value = 0;
    if(abs(lr_value) < GIMBAL_DEAD_ZONE) lr_value = 0;

    // 右摇杆控制云台
    if(ud_value != 0) {
        gimbal->target_pitch += (ud_value > 0) ? GIMBAL_SPEED : -GIMBAL_SPEED;
    }

    if(lr_value != 0) {
        gimbal->target_yaw += (lr_value > 0) ? GIMBAL_SPEED : -GIMBAL_SPEED;
    }
    if(gimbal->target_pitch < GIMBAL_PITCH_MIN) gimbal->target_pitch = GIMBAL_PITCH_MIN;
    if(gimbal->target_pitch > GIMBAL_PITCH_MAX) gimbal->target_pitch = GIMBAL_PITCH_MAX;
    if(gimbal->target_yaw < GIMBAL_YAW_MIN) gimbal->target_yaw = GIMBAL_YAW_MIN;
    if(gimbal->target_yaw > GIMBAL_YAW_MAX) gimbal->target_yaw = GIMBAL_YAW_MAX;
    // START按钮 - 复位云台
    if(btn1 & 0x08) {
        gimbal->target_pitch = 90;
        gimbal->target_yaw = 90;
    }
}
*/
#define JOY_CENTER 128
#define JOY_DEADZONE 15
#define GIMBAL_SPEED 2
#define FILTER_FACTOR 0.75f  // 滤波系数
void Gimbal_ControlWithPS2(Gimbal_TypeDef* gimbal, uint8_t RJoy_UD, uint8_t RJoy_LR, uint8_t btn1, uint8_t btn2)
{
    static float filtered_ud = 0, filtered_lr = 0;

    // 一阶低通滤波
    filtered_ud = FILTER_FACTOR * filtered_ud + (1 - FILTER_FACTOR) * (JOY_CENTER - (float)RJoy_UD);
    filtered_lr = FILTER_FACTOR * filtered_lr + (1 - FILTER_FACTOR) * ((float)RJoy_LR - JOY_CENTER);

    // 应用死区
    int16_t ud_value = (fabsf(filtered_ud) > JOY_DEADZONE) ? (int16_t)filtered_ud : 0;
    int16_t lr_value = (fabsf(filtered_lr) > JOY_DEADZONE) ? (int16_t)filtered_lr : 0;

    // 控制角度
    if(ud_value != 0) {
        gimbal->target_pitch += (ud_value > 0) ? GIMBAL_SPEED : -GIMBAL_SPEED;
    }

    if(lr_value != 0) {
        gimbal->target_yaw += (lr_value > 0) ? GIMBAL_SPEED : -GIMBAL_SPEED;
    }

    // 角度限制（立即执行）
    if(gimbal->target_pitch < GIMBAL_PITCH_MIN) gimbal->target_pitch = GIMBAL_PITCH_MIN;
    if(gimbal->target_pitch > GIMBAL_PITCH_MAX) gimbal->target_pitch = GIMBAL_PITCH_MAX;
    if(gimbal->target_yaw < GIMBAL_YAW_MIN) gimbal->target_yaw = GIMBAL_YAW_MIN;
    if(gimbal->target_yaw > GIMBAL_YAW_MAX) gimbal->target_yaw = GIMBAL_YAW_MAX;

    // START按钮 - 复位云台
    if(btn1 & 0x08) {
        gimbal->target_pitch = 90;
        gimbal->target_yaw = 90;
    }
}

// 更新云台位置

// 更新云台位置 - 直接移动到目标位置
void Gimbal_Update(Gimbal_TypeDef* gimbal)
{
    // 俯仰舵机
    if(gimbal->current_pitch != gimbal->target_pitch) {
        gimbal->current_pitch = gimbal->target_pitch;
        __HAL_TIM_SET_COMPARE(gimbal->pwm_tim, TIM_CHANNEL_1, AngleToCompare(gimbal->current_pitch));
    }

    // 偏航舵机
    if(gimbal->current_yaw != gimbal->target_yaw) {
        gimbal->current_yaw = gimbal->target_yaw;
        __HAL_TIM_SET_COMPARE(gimbal->pwm_tim, TIM_CHANNEL_2, AngleToCompare(gimbal->current_yaw));
    }
}


/*
void Gimbal_Update(Gimbal_TypeDef* gimbal)
{
    // 平滑移动到目标位置
    if(abs(gimbal->current_pitch - gimbal->target_pitch) > 1) {
        if(gimbal->current_pitch < gimbal->target_pitch) {
            gimbal->current_pitch++;
        } else {
            gimbal->current_pitch--;
        }
        __HAL_TIM_SET_COMPARE(gimbal->pwm_tim, TIM_CHANNEL_1, AngleToCompare(gimbal->current_pitch));
    }

    if(abs(gimbal->current_yaw - gimbal->target_yaw) > 1) {
        if(gimbal->current_yaw < gimbal->target_yaw) {
            gimbal->current_yaw++;
        } else {
            gimbal->current_yaw--;
        }
        __HAL_TIM_SET_COMPARE(gimbal->pwm_tim, TIM_CHANNEL_2, AngleToCompare(gimbal->current_yaw));
    }
}
*/
// 水弹发射初始化
void WaterBullet_Init(WaterBullet_TypeDef* shooter)
{
    shooter->is_shooting = 0;
    // 初始化GPIO - 确保电机初始状态为停止
    HAL_GPIO_WritePin(WATER_BULLET_PORT, WATER_BULLET_PIN, GPIO_PIN_RESET);
}

// 基于PS2控制水弹发射 - 使用△按钮，按住发射，松开停止
void WaterBullet_ControlWithPS2(WaterBullet_TypeDef* shooter, uint8_t btn2) {
    static uint8_t last_btn2_state = 0;
    uint8_t current_btn2_state = btn2 & 0x10;

    // 检测上升沿（按下）
    if(current_btn2_state && !last_btn2_state) {
        shooter->is_shooting = 1;
        HAL_GPIO_WritePin(WATER_BULLET_PORT, WATER_BULLET_PIN, GPIO_PIN_SET);
    }
    // 检测下降沿（松开）
    else if(!current_btn2_state && last_btn2_state) {
        shooter->is_shooting = 0;
        HAL_GPIO_WritePin(WATER_BULLET_PORT, WATER_BULLET_PIN, GPIO_PIN_RESET);
    }

    last_btn2_state = current_btn2_state;
}