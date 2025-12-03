#ifndef __MOVE_H
#define __MOVE_H

#include "stm32f1xx_hal.h"

// 电机引脚定义
// 左前电机
#define MOTOR_LF_PWM_PIN     GPIO_PIN_8     // PA8 - TIM1_CH1 (ENA)
#define MOTOR_LF_IN1_PIN     GPIO_PIN_12    // PB12 (IN1)
#define MOTOR_LF_IN2_PIN     GPIO_PIN_13    // PB13 (IN2)

// 右前电机
#define MOTOR_RF_PWM_PIN     GPIO_PIN_9     // PA9 - TIM1_CH2 (ENB)
#define MOTOR_RF_IN1_PIN     GPIO_PIN_14    // PB14 (IN3)
#define MOTOR_RF_IN2_PIN     GPIO_PIN_15    // PB15 (IN4)

// 左后电机
#define MOTOR_LB_PWM_PIN     GPIO_PIN_10    // PA10 - TIM1_CH3 (ENA)
#define MOTOR_LB_IN1_PIN     GPIO_PIN_0     // PB0 (IN1)
#define MOTOR_LB_IN2_PIN     GPIO_PIN_1     // PB1 (IN2)

// 右后电机
#define MOTOR_RB_PWM_PIN     GPIO_PIN_11    // PA11 - TIM1_CH4 (ENB)
#define MOTOR_RB_IN1_PIN     GPIO_PIN_2    // PB2 (IN3)
#define MOTOR_RB_IN2_PIN     GPIO_PIN_3    // PB3 (IN4)

// 电机结构体
typedef struct {
    GPIO_TypeDef* in1_port;      // IN1端口
    uint16_t in1_pin;            // IN1引脚
    GPIO_TypeDef* in2_port;      // IN2端口
    uint16_t in2_pin;            // IN2引脚
    TIM_HandleTypeDef* pwm_tim;  // PWM定时器
    uint32_t pwm_channel;        // PWM通道
    int16_t speed;               // -255 to 255
} Motor_TypeDef;

// 运动控制结构体
typedef struct {
    Motor_TypeDef lf_motor;  // 左前电机
    Motor_TypeDef rf_motor;  // 右前电机
    Motor_TypeDef lb_motor;  // 左后电机
    Motor_TypeDef rb_motor;  // 右后电机
} Motion_TypeDef;

// 运动模式枚举
typedef enum {
    MODE_STOP = 0,
    MODE_FORWARD,
    MODE_BACKWARD,
    MODE_LEFT,
    MODE_RIGHT,
    MODE_FORWARD_LEFT,
    MODE_FORWARD_RIGHT,
    MODE_BACKWARD_LEFT,
    MODE_BACKWARD_RIGHT,
    MODE_SPIN_LEFT,
    MODE_SPIN_RIGHT
} Motion_Mode;

// 函数声明
void Motion_Init(Motion_TypeDef* motion);
void Motion_SetSpeed(Motion_TypeDef* motion, int16_t lf_speed, int16_t rf_speed, int16_t lb_speed, int16_t rb_speed);
void Motion_Stop(Motion_TypeDef* motion);
void Motion_Move(Motion_TypeDef* motion, Motion_Mode mode, uint8_t speed);
void Motion_ControlWithPS2(Motion_TypeDef* motion, uint8_t LJoy_UD, uint8_t LJoy_LR, uint8_t btn1, uint8_t btn2);

#endif