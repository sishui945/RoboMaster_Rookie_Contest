/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ax_delay.h"
#include "ax_ps2.h"
#include "ax_uart1.h"
#include "claw.h"
#include "debug_module.h"
#include "gimbal.h"
#include "move.h"
// #include "bluetooth.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
JOYSTICK_TypeDef joystick;
Motion_TypeDef motion;
Gimbal_TypeDef gimbal;
WaterBullet_TypeDef water_bullet;
Claw_TypeDef claw;
static uint32_t last_update_time = 0;
// uint16_t battery_voltage = 0; // 或一个合理的初始值，如 12000
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void System_Init(void);

void PWM_Init();

void Debug_Output_Status(void);

void Steering_Engine_360(uint8_t direction, uint8_t speed);

static void DWT_Init(void);

void All_System_Center(void);

static void DWT_Delay_us(uint32_t us);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;

}
int fgetc(FILE *f) {
    uint8_t ch;
    HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
    return ch;
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    System_Init();
     /*调试模块初始化 */
    Debug_Module_Init(&debug_instance, &huart1);
    /* 设置输出级别为详细 */
    Debug_Module_SetLevel(&debug_instance, DEBUG_LEVEL_VERBOSE);
    /* 输出启动信息 */
    DEBUG_INFO("System Initialized");
    DEBUG_INFO("Clock: %lu Hz", HAL_RCC_GetHCLKFreq());
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    HAL_Delay(1000);
    while (1) {
        AX_PS2_ScanKey(&joystick);

        uint32_t current_time = HAL_GetTick();
        if (joystick.mode == 0x41 || joystick.mode == 0x73 || joystick.mode == 0x79) {
            /*if(current_time - last_update_time >= 20) {

            }
            if(current_time - last_update_time >= 10) {
                Gimbal_Update(&gimbal);
                last_update_time = current_time;
            }*/
            // 运动控制
            Motion_ControlWithPS2(&motion,
                                  joystick.LJoy_UD,
                                  joystick.LJoy_LR,
                                  joystick.btn1,
                                  joystick.btn2);


            // 云台控制
            Gimbal_ControlWithPS2(&gimbal,
                              joystick.RJoy_UD,
                              joystick.RJoy_LR,
                              joystick.btn1,
                              joystick.btn2);
            last_update_time = current_time;

            // 水弹发射控制 - △按钮，按住发射
            WaterBullet_ControlWithPS2(&water_bullet, joystick.btn2);

            // 抓取控制 - ○按钮切换抓取，L2/R2控制升降
            Claw_ControlWithPS2(&claw, joystick.btn2);
        } else {
            // PS2断开保护
            Motion_Stop(&motion);
            WaterBullet_ControlWithPS2(&water_bullet, 0); // 发送0确保停止发射
            Claw_Lift_Stop(&claw); // 停止抓取机构
        }
        // 更新所有系统
        Gimbal_Update(&gimbal);
        Claw_Update(&claw);
        Debug_Output_RobotStatus(&joystick, &motion, &gimbal, &claw, &water_bullet);
        // Debug_Output_Status();
        HAL_Delay(20);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 所有机构归中位置
void All_System_Center(void) {
    // 停止所有运动
    Motion_Stop(&motion);

    // 云台归中
    Gimbal_SetAngle(&gimbal, 90, 90);
    while(abs(gimbal.current_pitch - 90) > 2 ||
          abs(gimbal.current_yaw - 90) > 2) {
        Gimbal_Update(&gimbal);
        HAL_Delay(10);
          }

    // 机械臂归中
    Claw_Grip_Open(&claw);
    Claw_Lift_Stop(&claw);

    // 水弹停止
    WaterBullet_ControlWithPS2(&water_bullet, 0);

    HAL_Delay(500); // 等待稳定
}
void System_Init(void) {
    PWM_Init();
    AX_PS2_Init();
    AX_DELAY_Init();
    AX_UART1_Init(115200);
    DWT_Init();
    Motion_Init(&motion);
    Gimbal_Init(&gimbal, &htim2); // 使用TIM2控制云台舵机
    WaterBullet_Init(&water_bullet);
    Claw_Init(&claw, &htim2);
    //归中
    All_System_Center();
}
/*
static uint8_t uart_busy = 0;
void Debug_Output_Status(void) {
    static uint32_t last_debug = 0;

    char buff[200];  // 增大缓冲区确保安全

    // 检查UART是否空闲，避免数据覆盖
    if (uart_busy) {
        return;
    }

    // 每500ms输出一次状态
    if (HAL_GetTick() - last_debug > 500) {
        // 一次性构建所有输出
        int len = snprintf(buff, sizeof(buff),
            "=== Robot Status ===\r\n"
            "Motion: LF=%d RF=%d LB=%d RB=%d\r\n"
            "Gimbal: Pitch=%d Yaw=%d\r\n"
            "Claw: State=%d Lifting=%d\r\n"
            "PS2: Connected=%d Mode=0x%02X\r\n"
            "===================\r\n\r\n",
            motion.lf_motor.speed, motion.rf_motor.speed,
            motion.lb_motor.speed, motion.rb_motor.speed,
            gimbal.current_pitch, gimbal.current_yaw,
            claw.grip_state, claw.is_lifting,
            (joystick.mode == 0x41 || joystick.mode == 0x73 || joystick.mode == 0x79),
            joystick.mode);

        if (len > 0 && len < sizeof(buff)) {
            uart_busy = 1;
            HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, len);

            if (status != HAL_OK) {
                uart_busy = 0;  // 传输失败，重置标志
            } else {
                last_debug = HAL_GetTick();
            }
        }
    }
}

// 需要在UART传输完成中断回调中重置busy标志
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart1.Instance) {
        uart_busy = 0;  // 重置忙标志
    }
}
*/
void PWM_Init() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * 72; // 72 MHz
    while ((DWT->CYCCNT - start) < cycles);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    // 禁用中断，停止所有可能的中断触发
    __disable_irq();

    // 停止所有电机和动作以确保安全
    Motion_Stop(&motion);
    Claw_Lift_Stop(&claw);

    // 错误指示：快速闪烁PB12灯
    while (1) {
        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);// 切换LED状态
        HAL_Delay(200); // 每200ms闪烁一次
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
