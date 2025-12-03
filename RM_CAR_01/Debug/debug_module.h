#ifndef __DEBUG_MODULE_H
#define __DEBUG_MODULE_H
#include "main.h"
#include <string.h>

#include "ax_ps2.h"
#include "claw.h"
#include "gimbal.h"
#include "move.h"

/* 调试模块配置 */
#define DEBUG_BUFFER_SIZE    256     // DMA缓冲区大小
#define DEBUG_DMA_TIMEOUT    1000    // DMA传输超时(ms)
#define DEBUG_OUTPUT_PERIOD  500     // 状态输出周期(ms)

/* 调试输出级别 */
typedef enum {
    DEBUG_LEVEL_NONE = 0,
    DEBUG_LEVEL_ERROR,
    DEBUG_LEVEL_WARNING,
    DEBUG_LEVEL_INFO,
    DEBUG_LEVEL_VERBOSE
} DebugLevel_t;

/* 调试模块结构体 */
typedef struct {
    /* DMA缓冲区 */
    uint8_t tx_buffer[DEBUG_BUFFER_SIZE];
    
    /* 状态标志 */
    volatile uint8_t dma_busy;           // DMA传输忙标志
    volatile uint8_t last_transmit_ok;   // 上次传输成功标志
    uint32_t last_send_time;             // 上次发送时间
    uint32_t error_count;                // 错误计数
    
    /* UART句柄 */
    UART_HandleTypeDef* huart;
    
    /* 配置 */
    DebugLevel_t output_level;           // 输出级别
    uint8_t enable_output;              // 使能输出
} DebugModule_t;

/* 全局调试实例声明 */
extern DebugModule_t debug_instance;

/* 函数声明 */
void Debug_Module_Init(DebugModule_t* debug, UART_HandleTypeDef* huart);
void Debug_Module_SetLevel(DebugModule_t* debug, DebugLevel_t level);
void Debug_Module_Enable(DebugModule_t* debug, uint8_t enable);
/*
 JOYSTICK_TypeDef joystick;
Motion_TypeDef motion;
Gimbal_TypeDef gimbal;
WaterBullet_TypeDef water_bullet;
Claw_TypeDef claw;
 */
/* 核心输出函数 */
void Debug_Output_RobotStatus(JOYSTICK_TypeDef* joystick, Motion_TypeDef* motion,
    Gimbal_TypeDef* gimbal, Claw_TypeDef* claw, WaterBullet_TypeDef* shooter);
void Debug_Output_Formatted(DebugModule_t* debug, const char* format, ...);
void Debug_Output_String(DebugModule_t* debug, const char* str);
void Debug_Output_Buffer(DebugModule_t* debug, uint8_t* data, uint16_t length);

/* 辅助函数 */
void Debug_Flush(DebugModule_t* debug);
uint8_t Debug_IsReady(DebugModule_t* debug);
void Debug_WaitReady(DebugModule_t* debug, uint32_t timeout_ms);

/* 输出宏定义（方便使用） */
#define DEBUG_ERROR(fmt, ...)   do { if(debug_instance.output_level >= DEBUG_LEVEL_ERROR) \
    Debug_Output_Formatted(&debug_instance, "[ERROR] " fmt, ##__VA_ARGS__); } while(0)

#define DEBUG_WARN(fmt, ...)    do { if(debug_instance.output_level >= DEBUG_LEVEL_WARNING) \
    Debug_Output_Formatted(&debug_instance, "[WARN]  " fmt, ##__VA_ARGS__); } while(0)

#define DEBUG_INFO(fmt, ...)    do { if(debug_instance.output_level >= DEBUG_LEVEL_INFO) \
    Debug_Output_Formatted(&debug_instance, "[INFO]  " fmt, ##__VA_ARGS__); } while(0)

#define DEBUG_VERBOSE(fmt, ...) do { if(debug_instance.output_level >= DEBUG_LEVEL_VERBOSE) \
    Debug_Output_Formatted(&debug_instance, "[DEBUG] " fmt, ##__VA_ARGS__); } while(0)

#define DEBUG_HEX(data, len)    do { if(debug_instance.output_level >= DEBUG_LEVEL_VERBOSE) \
    Debug_Output_Hex(&debug_instance, data, len); } while(0)

#endif /* __DEBUG_MODULE_H */