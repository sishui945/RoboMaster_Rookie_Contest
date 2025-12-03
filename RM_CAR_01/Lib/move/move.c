#include "move.h"
#include "main.h"
#include <stdlib.h>
#include "tim.h"
/*
正转：IN1=1, IN2=0, PWM=速度值
反转：IN1=0, IN2=1, PWM=速度值
停止：IN1=0, IN2=0, PWM=0（或IN1=1, IN2=1刹车）
刹车：IN1=1, IN2=1, PWM=0
 */

#define DEAD_ZONE 30 //死区范围
#define PWM_MAX 255  //最大PWM值
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/*
 * 初始化电机
 * 参数： motion: 运动控制结构体指针
 * 初始化四个电机的引脚和PWM通道
 */
void Motion_Init(Motion_TypeDef* motion)
{
    // 初始化左前电机
    motion->lf_motor.in1_port = GPIOB;
    motion->lf_motor.in1_pin = MOTOR_LF_IN1_PIN;
    motion->lf_motor.in2_port = GPIOB;
    motion->lf_motor.in2_pin = MOTOR_LF_IN2_PIN;
    motion->lf_motor.pwm_tim = &htim1;
    motion->lf_motor.pwm_channel = TIM_CHANNEL_1;
    motion->lf_motor.speed = 0;

    // 初始化右前电机
    motion->rf_motor.in1_port = GPIOB;
    motion->rf_motor.in1_pin = MOTOR_RF_IN1_PIN;
    motion->rf_motor.in2_port = GPIOB;
    motion->rf_motor.in2_pin = MOTOR_RF_IN2_PIN;
    motion->rf_motor.pwm_tim = &htim1;
    motion->rf_motor.pwm_channel = TIM_CHANNEL_2;
    motion->rf_motor.speed = 0;

    // 初始化左后电机
    motion->lb_motor.in1_port = GPIOB;
    motion->lb_motor.in1_pin = MOTOR_LB_IN1_PIN;
    motion->lb_motor.in2_port = GPIOB;
    motion->lb_motor.in2_pin = MOTOR_LB_IN2_PIN;
    motion->lb_motor.pwm_tim = &htim1;
    motion->lb_motor.pwm_channel = TIM_CHANNEL_3;
    motion->lb_motor.speed = 0;

    // 初始化右后电机
    motion->rb_motor.in1_port = GPIOB;
    motion->rb_motor.in1_pin = MOTOR_RB_IN1_PIN;
    motion->rb_motor.in2_port = GPIOB;
    motion->rb_motor.in2_pin = MOTOR_RB_IN2_PIN;
    motion->rb_motor.pwm_tim = &htim1;
    motion->rb_motor.pwm_channel = TIM_CHANNEL_4;
    motion->rb_motor.speed = 0;

    // 启动PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    //停止状态
    Motion_Stop(motion);
}

// 设置单个电机的速度和方向
static void Motor_SetSpeed(Motor_TypeDef* motor, int16_t speed) {
    // 限制速度范围
    if(speed > PWM_MAX) speed = PWM_MAX;
    if(speed < -PWM_MAX) speed = -PWM_MAX;
    //设置速度值
    motor->speed = speed;

    if(speed > 0) {
        // 正转: IN1=1, IN2=0, PWM=速度值
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, speed);
    } else if(speed < 0) {
        // 反转: IN1=0, IN2=1, PWM=速度值
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, -speed);
    } else {
        // 停止: IN1=0, IN2=0, PWM=0 (或IN1=1, IN2=1也可刹车)
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, 0);
    }
}

// 设置所有电机速度
void Motion_SetSpeed(Motion_TypeDef* motion, int16_t lf_speed, int16_t rf_speed, int16_t lb_speed, int16_t rb_speed)
{
    Motor_SetSpeed(&motion->lf_motor, lf_speed);
    Motor_SetSpeed(&motion->rf_motor, rf_speed);
    Motor_SetSpeed(&motion->lb_motor, lb_speed);
    Motor_SetSpeed(&motion->rb_motor, rb_speed);
}

// 停止所有电机
void Motion_Stop(Motion_TypeDef* motion)
{
    Motion_SetSpeed(motion, 0, 0, 0, 0);
}

// 根据运动模式移动
// 参数： motion: 运动控制结构体指针
//        mode: 运动模式
//        speed: 速度值 (0 - PWM_MAX)
// 可以根据模式直接设置，不过函数只用了ODE_SPIN_LEFT与MODE_SPIN_RIGHT这两个模式，其他模式未使用
void Motion_Move(Motion_TypeDef* motion, Motion_Mode mode, uint8_t speed)
{
    switch(mode) {
        /*
        case MODE_STOP:
            Motion_SetSpeed(motion, 0, 0, 0, 0);
            break;

        case MODE_FORWARD:
            Motion_SetSpeed(motion, speed, speed, speed, speed);
            break;

        case MODE_BACKWARD:
            Motion_SetSpeed(motion, -speed, -speed, -speed, -speed);
            break;

        case MODE_LEFT:
            // 左转 - 左侧后退，右侧前进
            Motion_SetSpeed(motion, -speed, speed, -speed, speed);
            break;

        case MODE_RIGHT:
            // 右转 - 左侧前进，右侧后退
            Motion_SetSpeed(motion, speed, -speed, speed, -speed);
            break;

        case MODE_FORWARD_LEFT:
            // 左前方移动
            Motion_SetSpeed(motion, 0, speed, speed, 0);
            break;

        case MODE_FORWARD_RIGHT:
            // 右前方移动
            Motion_SetSpeed(motion, speed, 0, 0, speed);
            break;

        case MODE_BACKWARD_LEFT:
            // 左后方移动
            Motion_SetSpeed(motion, -speed, 0, 0, -speed);
            break;

        case MODE_BACKWARD_RIGHT:
            // 右后方移动
            Motion_SetSpeed(motion, 0, -speed, -speed, 0);
            break;
*/
        case MODE_SPIN_LEFT:
            // 原地左转
            Motion_SetSpeed(motion, -speed, speed, -speed, speed);
            break;

        case MODE_SPIN_RIGHT:
            // 原地右转
            Motion_SetSpeed(motion, speed, -speed, speed, -speed);
            break;
    }
}
// 基于PS2手柄控制
//只用到了遥感与L1、R1键控制原地转向
//未来可加入按钮
void Motion_ControlWithPS2(Motion_TypeDef* motion, uint8_t LJoy_UD, uint8_t LJoy_LR, uint8_t btn1, uint8_t btn2)
{
    static uint8_t special_mode = 0;

    // 检查特殊按钮控制
    if(btn2 & 4) { // L1 - 原地左转
        Motion_Move(motion, MODE_SPIN_LEFT, 150);
        special_mode = 1;
    } else if(btn2 & 8) { // R1 - 原地右转
        Motion_Move(motion, MODE_SPIN_RIGHT, 150);
        special_mode = 1;
    } else {
        special_mode = 0;
    }

    // 只有没有特殊模式时才使用摇杆控制
    if(!special_mode) {
        // 摇杆处理
        //LJoy_UD与LJoy_LR与的范围为0x00-0xFF，中间值为0x80(128)
        int16_t ud_value = 128 - (int16_t)LJoy_UD;  // 上=正，下=负
        int16_t lr_value = (int16_t)LJoy_LR - 128;  // 右=正，左=负

        // 应用死区
        if(abs(ud_value) < DEAD_ZONE) ud_value = 0;
        if(abs(lr_value) < DEAD_ZONE) lr_value = 0;

        if(ud_value != 0 || lr_value != 0) {
            // 映射到速度值
            int16_t forward_speed = (ud_value * PWM_MAX) / 128;
            int16_t turn_speed = (lr_value * PWM_MAX) / 128;

            // 差速转向计算
            int16_t left_speed = forward_speed - turn_speed;
            int16_t right_speed = forward_speed + turn_speed;

            // 限幅
            if(left_speed > PWM_MAX) left_speed = PWM_MAX;
            if(left_speed < -PWM_MAX) left_speed = -PWM_MAX;
            if(right_speed > PWM_MAX) right_speed = PWM_MAX;
            if(right_speed < -PWM_MAX) right_speed = -PWM_MAX;

            Motion_SetSpeed(motion, left_speed, right_speed, left_speed, right_speed);
        } else {
            // 摇杆回中时停止
            Motion_Stop(motion);
        }
    }
}