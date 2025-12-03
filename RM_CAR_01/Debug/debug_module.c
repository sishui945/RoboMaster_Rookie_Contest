#include "debug_module.h"
#include <stdio.h>
#include <stdarg.h>

#include "ax_ps2.h"

/* 全局调试实例定义 */
DebugModule_t debug_instance;

/*/* 机器人状态结构体（需要根据实际项目定义） #1#
typedef struct {
    struct {
        int16_t speed;
    } lf_motor, rf_motor, lb_motor, rb_motor;
} Motion_t;

typedef struct {
    int16_t current_pitch;
    int16_t current_yaw;
} Gimbal_t;

typedef struct {
    uint8_t grip_state;
    uint8_t is_lifting;
} Claw_t;

typedef struct {
    uint8_t mode;
} Joystick_t;

/* 假设这些是全局变量（在实际项目中需要正确声明） #1#
extern Motion_t motion;
extern Gimbal_t gimbal;
extern Claw_t claw;

/* 私有函数声明 */
static void Debug_Transmit_DMA(DebugModule_t* debug, uint16_t length);
static void Debug_Check_Timeout(DebugModule_t* debug);

/* 模块初始化 */
void Debug_Module_Init(DebugModule_t* debug, UART_HandleTypeDef* huart)
{
    if(debug == NULL || huart == NULL) return;

    /* 初始化结构体 */
    memset(debug, 0, sizeof(DebugModule_t));

    /* 设置UART句柄 */
    debug->huart = huart;

    /* 初始化缓冲区 */
    memset(debug->tx_buffer, 0, DEBUG_BUFFER_SIZE);

    /* 默认配置 */
    debug->output_level = DEBUG_LEVEL_INFO;  // 默认输出INFO级别
    debug->enable_output = 1;                // 默认使能输出
    debug->last_transmit_ok = 1;

    DEBUG_INFO("Debug Module Initialized");
}

/* 设置输出级别 */
void Debug_Module_SetLevel(DebugModule_t* debug, DebugLevel_t level)
{
    if(debug) {
        debug->output_level = level;
    }
}

/* 使能/禁用输出 */
void Debug_Module_Enable(DebugModule_t* debug, uint8_t enable)
{
    if(debug) {
        debug->enable_output = enable;
    }
}

/* 检查并开始DMA传输 */
static void Debug_Transmit_DMA(DebugModule_t* debug, uint16_t length)
{
    if(debug == NULL || debug->huart == NULL) return;
    if(length == 0 || length > DEBUG_BUFFER_SIZE) return;

    /* 检查DMA是否繁忙 */
    if(debug->dma_busy) {
        Debug_Check_Timeout(debug);
        if(debug->dma_busy) {
            return;  // DMA仍然繁忙
        }
    }

    /* 检查UART是否就绪 */
    if(__HAL_UART_GET_FLAG(debug->huart, UART_FLAG_TXE)) {
        debug->dma_busy = 1;

        /* 开始DMA传输 */
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(debug->huart,
                                                        debug->tx_buffer,
                                                        length);

        if(status != HAL_OK) {
            debug->dma_busy = 0;
            debug->error_count++;
            debug->last_transmit_ok = 0;
        } else {
            debug->last_send_time = HAL_GetTick();
            debug->last_transmit_ok = 1;
        }
    }
}

/* 检查DMA传输超时 */
static void Debug_Check_Timeout(DebugModule_t* debug)
{
    if(debug->dma_busy) {
        /* 检查是否超时 */
        if(HAL_GetTick() - debug->last_send_time > DEBUG_DMA_TIMEOUT) {
            /* 中止传输并重置状态 */
            HAL_UART_AbortTransmit(debug->huart);
            debug->dma_busy = 0;
            debug->error_count++;
            debug->last_transmit_ok = 0;

            /* 可选：重新初始化UART/DMA */
            // HAL_UART_DeInit(debug->huart);
            // HAL_UART_Init(debug->huart);
        }
    }
}

/* DMA传输完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(debug_instance.huart != NULL &&
       huart->Instance == debug_instance.huart->Instance)
    {
        debug_instance.dma_busy = 0;

        /* 可选：自动清除缓冲区 */
        // memset(debug_instance.tx_buffer, 0, DEBUG_BUFFER_SIZE);
    }
}

/* DMA传输错误回调 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(debug_instance.huart != NULL &&
       huart->Instance == debug_instance.huart->Instance)
    {
        debug_instance.dma_busy = 0;
        debug_instance.error_count++;
        debug_instance.last_transmit_ok = 0;

        /* 清除错误标志 */
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);

        /* 可选：重新初始化UART */
        // HAL_UART_DeInit(huart);
        // HAL_UART_Init(huart);

        DEBUG_ERROR("UART Error: 0x%08X", huart->ErrorCode);
    }
}

/* 格式化输出函数 */
void Debug_Output_Formatted(DebugModule_t* debug, const char* format, ...)
{
    if(debug == NULL || !debug->enable_output) return;

    /* 检查DMA状态 */
    Debug_Check_Timeout(debug);
    if(debug->dma_busy) return;

    /* 格式化字符串 */
    va_list args;
    va_start(args, format);
    int length = vsnprintf((char*)debug->tx_buffer,
                          DEBUG_BUFFER_SIZE,
                          format,
                          args);
    va_end(args);

    /* 添加换行符（如果缓冲区足够） */
    if(length > 0 && length < DEBUG_BUFFER_SIZE - 2) {
        debug->tx_buffer[length++] = '\r';
        debug->tx_buffer[length++] = '\n';
    }

    /* 开始传输 */
    if(length > 0 && length < DEBUG_BUFFER_SIZE) {
        Debug_Transmit_DMA(debug, length);
    }
}

/* 输出字符串 */
void Debug_Output_String(DebugModule_t* debug, const char* str)
{
    if(debug == NULL || !debug->enable_output || str == NULL) return;

    Debug_Check_Timeout(debug);
    if(debug->dma_busy) return;

    size_t length = strlen(str);
    if(length > 0 && length < DEBUG_BUFFER_SIZE) {
        strncpy((char*)debug->tx_buffer, str, DEBUG_BUFFER_SIZE - 1);
        Debug_Transmit_DMA(debug, length);
    }
}

/* 输出二进制数据（十六进制格式） */
void Debug_Output_Hex(DebugModule_t* debug, uint8_t* data, uint16_t length)
{
    if(debug == NULL || !debug->enable_output || data == NULL) return;

    Debug_Check_Timeout(debug);
    if(debug->dma_busy) return;

    /* 计算需要的缓冲区大小：每字节3字符（2十六进制+空格），加换行符 */
    uint16_t max_len = (DEBUG_BUFFER_SIZE - 10) / 3;
    if(length > max_len) length = max_len;

    int pos = snprintf((char*)debug->tx_buffer, DEBUG_BUFFER_SIZE, "HEX[%d]: ", length);

    for(uint16_t i = 0; i < length && pos < DEBUG_BUFFER_SIZE - 5; i++) {
        pos += snprintf((char*)debug->tx_buffer + pos,
                       DEBUG_BUFFER_SIZE - pos,
                       "%02X ", data[i]);
    }

    if(pos < DEBUG_BUFFER_SIZE - 2) {
        debug->tx_buffer[pos++] = '\r';
        debug->tx_buffer[pos++] = '\n';
    }

    Debug_Transmit_DMA(debug, pos);
}

/* 机器人状态输出（周期性调用） */
void Debug_Output_RobotStatus(JOYSTICK_TypeDef* joystick,
    Motion_TypeDef* motion,
    Gimbal_TypeDef* gimbal,
    Claw_TypeDef* claw, WaterBullet_TypeDef* shooter)
{
    static uint32_t last_output_time = 0;
    uint32_t current_time = HAL_GetTick();

    if(!debug_instance.enable_output) return;

    /* 检查输出周期 */
    if(current_time - last_output_time < DEBUG_OUTPUT_PERIOD) return;

    Debug_Check_Timeout(&debug_instance);
    if(debug_instance.dma_busy) return;

    /* 构建状态信息 */
    int pos = 0;

    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "\r\n=== Robot Status [%lu ms] ===\r\n", current_time);
    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "JOYSTICKCODE,bt1=%d,bt2=%d:\r\n", joystick->btn1, joystick->btn2);
    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "Motion: LF=%d RF=%d LB=%d RB=%d\r\n",
                   motion->lf_motor.speed, motion->rf_motor.speed,
                   motion->lb_motor.speed, motion->rb_motor.speed);

    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "Gimbal: Pitch=%d Yaw=%d\r\n",
                   gimbal->current_pitch, gimbal->current_yaw);
    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "is_shoott=%d\r\n",
                   shooter->is_shooting);
/*
uint8_t current_pitch;     // 当前俯仰角度
uint8_t current_yaw;       // 当前偏航角度
 */
    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "Claw: State=%d Lifting=%d\r\n",
                   claw->grip_state, claw->is_lifting);

/*
    *     ClawState_TypeDef grip_state;      // 抓取状态
        uint8_t is_lifting;                // 是否正在升降
 */
    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "PS2: Connected=%d Mode=0x%02X\r\n",
                   (joystick->mode == 0x41 || joystick->mode == 0x73 || joystick->mode == 0x79),
                   joystick->mode);

    /*pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "Battery: %d mV\r\n", battery_voltage);*/

    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "Debug: Errors=%lu Busy=%d\r\n",
                   debug_instance.error_count, debug_instance.dma_busy);

    pos += snprintf((char*)debug_instance.tx_buffer + pos,
                   DEBUG_BUFFER_SIZE - pos,
                   "===================\r\n");

    /* 开始传输 */
    if(pos > 0 && pos < DEBUG_BUFFER_SIZE) {
        Debug_Transmit_DMA(&debug_instance, pos);
        last_output_time = current_time;
    }
}

/* 等待DMA就绪 */
void Debug_WaitReady(DebugModule_t* debug, uint32_t timeout_ms)
{
    if(debug == NULL) return;

    uint32_t start_time = HAL_GetTick();
    while(debug->dma_busy) {
        if(HAL_GetTick() - start_time > timeout_ms) {
            Debug_Check_Timeout(debug);
            break;
        }
        // 短暂延时，避免CPU占用过高
        HAL_Delay(1);
    }
}

/* 检查DMA是否就绪 */
uint8_t Debug_IsReady(DebugModule_t* debug)
{
    if(debug == NULL) return 0;

    Debug_Check_Timeout(debug);
    return (debug->dma_busy == 0);
}

/* 刷新输出（等待当前传输完成） */
void Debug_Flush(DebugModule_t* debug)
{
    Debug_WaitReady(debug, DEBUG_DMA_TIMEOUT);
}