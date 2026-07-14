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
#include "SEGGER_RTT.h"
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

/* Cortex-M4 故障状态寄存器地址 */
#define SCB_CFSR    (*((volatile uint32_t *)0xE000ED28u))  /* 可配置故障状态寄存器 */
#define SCB_HFSR    (*((volatile uint32_t *)0xE000ED2Cu))  /* 硬故障状态寄存器 */
#define SCB_MMFAR   (*((volatile uint32_t *)0xE000ED34u))  /* 存储管理故障地址寄存器 */
#define SCB_BFAR    (*((volatile uint32_t *)0xE000ED38u))  /* 总线故障地址寄存器 */

/**
 * @brief  从异常堆栈帧中提取故障发生时的 PC 值
 * @param[out] pc  故障发生时的指令地址
 * @note   利用 LR（EXC_RETURN）的 bit2 判断使用 MSP 还是 PSP
 */
#define FAULT_GET_EXC_RETURN(exc_return_)                  \
    do {                                                    \
        register uint32_t lr_reg_ __ASM("lr");              \
        (exc_return_) = lr_reg_;                            \
    } while (0)

static uint32_t FaultGetStackedPc(uint32_t exc_return)
{
    uint32_t sp = ((exc_return & 0x04u) == 0u) ? __get_MSP() : __get_PSP();
    return *((uint32_t *)(sp + 0x18u));
}

#define FAULT_GET_PC(pc_, exc_return_)                      \
    do {                                                    \
        (pc_) = FaultGetStackedPc(exc_return_);              \
    } while (0)

/**
 * @brief  解析并输出 CFSR/HFSR 故障状态寄存器内容
 * @param[in] name       故障名称字符串
 * @param[in] fault_pc   故障指令地址
 * @param[in] exc_return EXC_RETURN 值
 */
static void FaultPrintRegs(const char *name, uint32_t fault_pc, uint32_t exc_return)
{
    uint32_t cfsr = SCB_CFSR;
    uint32_t hfsr = SCB_HFSR;
    uint32_t mmfar = SCB_MMFAR;
    uint32_t bfar = SCB_BFAR;

    SEGGER_RTT_WriteString(0, "\r\n=== ");
    SEGGER_RTT_WriteString(0, name);
    SEGGER_RTT_WriteString(0, " ===\r\n");
    SEGGER_RTT_printf(0, "PC=0x%08lX EXC_RETURN=0x%08lX\r\n", (unsigned long)fault_pc, (unsigned long)exc_return);
    SEGGER_RTT_printf(0, "CFSR=0x%08lX HFSR=0x%08lX\r\n", (unsigned long)cfsr, (unsigned long)hfsr);

    /* 解析 MMFSR (CFSR[7:0]) */
    if (cfsr & 0xFFu)
    {
        uint32_t mmfsr = cfsr & 0xFFu;
        SEGGER_RTT_printf(0, "MMFSR=0x%02lX", (unsigned long)mmfsr);
        if (mmfsr & 0x01u) SEGGER_RTT_WriteString(0, " IACCVIOL");   /* 指令访问违例 */
        if (mmfsr & 0x02u) SEGGER_RTT_WriteString(0, " DACCVIOL");   /* 数据访问违例 */
        if (mmfsr & 0x08u) SEGGER_RTT_WriteString(0, " MUNSTKERR");  /* 出栈时存储管理错误 */
        if (mmfsr & 0x10u) SEGGER_RTT_WriteString(0, " MSTKERR");    /* 入栈时存储管理错误 */
        if (mmfsr & 0x80u) SEGGER_RTT_printf(0, " MMFAR=0x%08lX", (unsigned long)mmfar);
        SEGGER_RTT_WriteString(0, "\r\n");
    }

    /* 解析 BFSR (CFSR[15:8]) */
    if (cfsr & 0xFF00u)
    {
        uint32_t bfsr = (cfsr >> 8u) & 0xFFu;
        SEGGER_RTT_printf(0, "BFSR=0x%02lX", (unsigned long)bfsr);
        if (bfsr & 0x01u) SEGGER_RTT_WriteString(0, " IBUSERR");     /* 指令总线错误 */
        if (bfsr & 0x02u) SEGGER_RTT_WriteString(0, " PRECISERR");   /* 精确数据总线错误 */
        if (bfsr & 0x04u) SEGGER_RTT_WriteString(0, " IMPRECISERR"); /* 非精确数据总线错误 */
        if (bfsr & 0x08u) SEGGER_RTT_WriteString(0, " UNSTKERR");    /* 出栈时总线错误 */
        if (bfsr & 0x10u) SEGGER_RTT_WriteString(0, " STKERR");      /* 入栈时总线错误 */
        if (bfsr & 0x80u) SEGGER_RTT_printf(0, " BFAR=0x%08lX", (unsigned long)bfar);
        SEGGER_RTT_WriteString(0, "\r\n");
    }

    /* 解析 UFSR (CFSR[31:16]) */
    if (cfsr & 0xFFFF0000u)
    {
        uint32_t ufsr = (cfsr >> 16u) & 0xFFFFu;
        SEGGER_RTT_printf(0, "UFSR=0x%04lX", (unsigned long)ufsr);
        if (ufsr & 0x0001u) SEGGER_RTT_WriteString(0, " UNDEFINSTR"); /* 未定义指令 */
        if (ufsr & 0x0002u) SEGGER_RTT_WriteString(0, " INVSTATE");   /* 无效状态（EPSR 错误） */
        if (ufsr & 0x0004u) SEGGER_RTT_WriteString(0, " INVPC");      /* 无效 PC 加载 */
        if (ufsr & 0x0008u) SEGGER_RTT_WriteString(0, " NOCP");       /* 无协处理器 */
        if (ufsr & 0x0100u) SEGGER_RTT_WriteString(0, " DIVBYZERO");  /* 除零错误 */
        SEGGER_RTT_WriteString(0, "\r\n");
    }

    /* 解析 HFSR */
    if (hfsr & 0x40000000u) SEGGER_RTT_WriteString(0, "[FORCED] 可配置故障升级为硬故障\r\n");
    if (hfsr & 0x80000000u) SEGGER_RTT_WriteString(0, "[DEBUGEVT] 调试事件\r\n");

    SEGGER_RTT_WriteString(0, "=== ");
    SEGGER_RTT_WriteString(0, name);
    SEGGER_RTT_WriteString(0, " END ===\r\n");
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

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
  SEGGER_RTT_WriteString(0, "\r\n*** NMI_Handler ***\r\n");
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
  uint32_t fault_pc, exc_return;
  FAULT_GET_EXC_RETURN(exc_return);
  FAULT_GET_PC(fault_pc, exc_return);
  FaultPrintRegs("HardFault", fault_pc, exc_return);
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
  uint32_t fault_pc, exc_return;
  FAULT_GET_EXC_RETURN(exc_return);
  FAULT_GET_PC(fault_pc, exc_return);
  FaultPrintRegs("MemManage", fault_pc, exc_return);
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
  uint32_t fault_pc, exc_return;
  FAULT_GET_EXC_RETURN(exc_return);
  FAULT_GET_PC(fault_pc, exc_return);
  FaultPrintRegs("BusFault", fault_pc, exc_return);
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
  uint32_t fault_pc, exc_return;
  FAULT_GET_EXC_RETURN(exc_return);
  FAULT_GET_PC(fault_pc, exc_return);
  FaultPrintRegs("UsageFault", fault_pc, exc_return);
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
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */

  /* USER CODE END TIM7_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief TIM1 capture/compare interrupt; CH4 up-count compare is the ADC sample trigger.
  */
void TIM1_CC_IRQHandler(void)
{
  /*
  if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC4) != RESET) &&
      ((TIM1->CR1 & TIM_CR1_DIR) == 0u))
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
  }
  */

  HAL_TIM_IRQHandler(&htim1);
}

/* USER CODE END 1 */
