#include "ax_ps2.h"
#include "ax_delay.h"
#define DI()     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)      // 数据输入引脚

#define CMD_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)   // 命令位高
#define CMD_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET) // 命令位低

#define CS_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)    // CS拉高
#define CS_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)  // CS拉低

#define CLK_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)    // 时钟拉高
#define CLK_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)  // 时钟拉低

const uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t PS2_data[9] = {0};  // 接收的数据

/**
  * @简  述  PS2初始化
  * @参  数  无
  * @返回值  无
  */
void AX_PS2_Init(void)
{
    // 删除原有的GPIO初始化代码，因为CubeMX已经初始化了
    // 只需要设置初始状态
    CS_H();
    CMD_H();
    CLK_H();

    // 添加延时确保稳定
    HAL_Delay(100);
}

/**
  * @简  述  PS2数据读写函数
  * @参  数  cmd:要写入的命令
  * @返回值  读出数据
  */
static uint8_t PS2_ReadWriteData(uint8_t cmd)
{
    volatile uint8_t res = 0;
    volatile uint8_t ref;

    // 写入命令，并读取一个1字节数据
    for(ref = 0x01; ref > 0x00; ref <<= 1)
    {
        // 输出一位数据
        if(ref & cmd)
            CMD_H();
        else
            CMD_L();

        CLK_L();
        AX_Delayus(20);  // 稍微增加延时

        // 读取一位数据
        if(DI())
            res |= ref;

        CLK_H();
        AX_Delayus(20);  // 稍微增加延时
    }

    // 返回读出数据
    return res;
}

/**
  * @简  述  PS2获取按键及摇杆数值。
  * @参  数  *JoystickStruct 手柄键值结构体
  * @返回值  无
  */
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
    uint8_t i;
    static uint8_t error_count = 0;

    // 使能手柄
    CS_L();
    AX_Delayus(50);  // 添加CS拉低后的延时

    // 读取PS2数据
    for(i = 0; i < 9; i++)
    {
        PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
        AX_Delayus(20);
    }

    // 关闭使能
    CS_H();

    // 数据有效性检查
    if(PS2_data[1] == 0x41 || PS2_data[1] == 0x73 || PS2_data[1] == 0x79)
    {
        error_count = 0;
        // 数值传递
        JoystickStruct->mode = PS2_data[1];
        JoystickStruct->btn1 = ~PS2_data[3];
        JoystickStruct->btn2 = ~PS2_data[4];
        JoystickStruct->RJoy_LR = PS2_data[5];
        JoystickStruct->RJoy_UD = PS2_data[6];
        JoystickStruct->LJoy_LR = PS2_data[7];
        JoystickStruct->LJoy_UD = PS2_data[8];
    }
    else
    {
        error_count++;
        if(error_count > 10)
        {
            // 重新初始化
            AX_PS2_Init();
            error_count = 0;
        }
    }
}