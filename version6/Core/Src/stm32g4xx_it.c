/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_nucleo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim6;

/* v6: TIM6 ISR → 主循环 的按键边沿标志
 * 这些变量在 TIM6_DAC_IRQHandler ISR 中置位，在 main.c 的 Button_Update() 中读取并清除
 * 声明为 volatile 确保 ISR 和主循环之间的可见性（防止编译器优化缓存） */
volatile uint8_t g_btn_press_edge   = 0;   /* 1 = 检测到按下边沿（从松开变为按下，去抖后确认） */
volatile uint8_t g_btn_release_edge = 0;   /* 1 = 检测到松开边沿（从按下变为松开，去抖后确认） */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  /* 按钮以 BUTTON_MODE_GPIO 轮询方式初始化，此中断不会触发，无需调用 BSP_PB_IRQHandler */
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  TIM6 中断处理函数 — 5ms 按键采样 + 计数式去抖 + 边沿检测
  *
  * 【触发频率】每 5ms 触发一次（TIM6 配置：170MHz/17000/50 = 200Hz）
  *
  * 【算法说明】
  *   1. 读取 B1 按钮 GPIO 电平（active-HIGH，板载 PULLDOWN）
  *   2. 计数式去抖：当前读数与去抖后状态不同时递增计数器
  *      连续 4 次读数一致（4 × 5ms = 20ms）才确认状态变化
  *      这比简单的延时去抖更可靠，且不阻塞 ISR
  *   3. 边沿检测：比较前后两次去抖后状态，检测 0→1（按下）或 1→0（松开）
  *   4. 设置 volatile 标志供主循环 Button_Update() 消费
  *
  * 【性能】ISR 执行时间约 ~1µs @ 170MHz，远低于 5ms 周期
  *
  * 【ISR 设计原则】
  *   - 只设置 volatile uint8_t 标志，不调用 HAL 阻塞函数
  *   - 不做浮点运算
  *   - 不调用 printf/sprintf
  *   - 具体的事件判定（单击/双击/长按）留给主循环
  */
void TIM6_DAC_IRQHandler(void)
{
  /* 检查是否为 TIM6 更新中断 */
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) == RESET) return;
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

  /* --- 计数式去抖算法 ---
   * debounce_cnt：连续读数与当前稳定状态不同的次数
   * stable：去抖后的稳定状态（0=松开, 1=按下）
   * prev_stable：上一次的稳定状态（用于边沿检测）
   * 所有变量为 static，跨 ISR 调用保持 */
  static uint8_t debounce_cnt = 0;
  static uint8_t stable       = 0;   /* 0=released, 1=pressed（去抖后） */
  static uint8_t prev_stable  = 0;

  /* 读取 GPIO：NUCLEO-G491RE B1 按钮 active-HIGH（板载下拉电阻） */
  uint8_t raw = (BSP_PB_GetState(BUTTON_USER) != 0) ? 1 : 0;

  if (raw == stable) {
    /* 读数与当前稳定状态一致：重置计数器 */
    debounce_cnt = 0;
  } else {
    /* 读数与当前稳定状态不同：递增计数器 */
    debounce_cnt++;
    if (debounce_cnt >= 4) {     /* 4 × 5ms = 20ms 连续不同 → 确认状态变化 */
      stable       = raw;        /* 更新稳定状态 */
      debounce_cnt = 0;          /* 重置计数器 */
    }
  }

  /* --- 边沿检测 ---
   * press_edge:   前次松开 → 本次按下（0→1 上升沿）
   * release_edge: 前次按下 → 本次松开（1→0 下降沿）
   * 标志不会被 ISR 连续置位（stable 变化后 prev_stable 同步更新） */
  uint8_t press   = (!prev_stable && stable);    /* 上升沿：松开 → 按下 */
  uint8_t release = ( prev_stable && !stable);   /* 下降沿：按下 → 松开 */
  prev_stable = stable;                          /* 更新前次状态 */

  /* 设置边沿标志（主循环 Button_Update() 中原子读取并清除） */
  if (press)   g_btn_press_edge   = 1;
  if (release) g_btn_release_edge = 1;
}

/* USER CODE END 1 */
