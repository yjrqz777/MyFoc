/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task.h"
#include <stdio.h>
#include "bsp_adc.h"
#include "bsp_ws2812.h"
#include "user_button.h"
#include "user_display.h"
#include "user_motor.h"
#include "user_system.h"
#include "user_time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned int PT_TICK[TASK_MAX] = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f)
{
    (void)f;
    return ch;
}

int fgetc(FILE *f)
{
    (void)f;
    return 0;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t u8RttJscopeUpBuffer[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UsrDebugRttInit(void)
{
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    SEGGER_RTT_ConfigUpBuffer(1, "JScope_f32", u8RttJscopeUpBuffer,
                              sizeof(u8RttJscopeUpBuffer), SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    SEGGER_RTT_WriteString(0, "Motor control startup\r\n");
}



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
  MX_SPI3_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100u);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

// HAL_DAC_SetValue(&hdac1,
//                  DAC_CHANNEL_1,
//                  DAC_ALIGN_12B_R,
//                  2048u);
// HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
/* 等待DAC、U6和电流采样运放稳定 */
// HAL_Delay(100u);
  // BspAdcPreOffset();
  // BspAdcPreOffset();
  UsrDebugRttInit();
  // HAL_Delay(500u);
  BspAdcPreOffset();
  HAL_Delay(100u);
  /* Send the initial-state color before motor startup can block the task loop. */
  BspWs2812Init();
  if (BspWs2812WriteColor(32u, 20u, 0u) != HAL_OK)
  {
      SEGGER_RTT_WriteString(0, "WS2812 start failed\r\n");
  }
  HAL_Delay(100u);
  HAL_StatusTypeDef MotorStatus;

  UsrMotorInit();

  UsrMotorStart();

  HAL_TIM_Base_Start_IT(&htim7);


  /* Fixed-PWM PA5 test (do not enable while using the WS2812 DMA driver). */
// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // 启动PWM输出

// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 106);  // 设置约50%占空比

// HAL_Delay(1000u);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    PT_TASK_REG(0, UsrDisplayTask);
    PT_TASK_REG(1, UsrButtonTask);
    PT_TASK_REG(2, UsrSystemTask);
    PT_TASK_REG(3, UsrTimeTask);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t TickCount = 0u;
    if (htim->Instance == TIM7)
    {
        UsrMotorSpeedLoop();   /* 1kHz 速度环：编码器读取 + 速度PI */
        TASK_TICK_UPDATE();
        if (++TickCount == 5u)
        {
          TickCount = 0u;
          button_ticks();
        }

    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  uint32_t ipsr;
  uint32_t primask;
  uint32_t control;
  uint32_t msp;
  uint32_t psp;
  uint32_t icsr;

  primask = __get_PRIMASK();

  __disable_irq();
  __DSB();
  __ISB();

  ipsr    = __get_IPSR();
  control = __get_CONTROL();
  msp     = __get_MSP();
  psp     = __get_PSP();
  icsr    = SCB->ICSR;

  SEGGER_RTT_WriteString(0,
                         "\r\n"
                         "========================================\r\n"
                         "*** Error_Handler ***\r\n"
                         "========================================\r\n");

  SEGGER_RTT_printf(0,
                    "Tick       : %u ms\r\n",
                    (unsigned int)HAL_GetTick());

  SEGGER_RTT_printf(0,
                    "CPUID      : 0x%08X\r\n",
                    (unsigned int)SCB->CPUID);

  SEGGER_RTT_printf(0,
                    "IPSR       : 0x%08X\r\n",
                    (unsigned int)ipsr);

  if (ipsr == 0U)
  {
    SEGGER_RTT_WriteString(0,
                           "Context    : Thread mode\r\n");
  }
  else if (ipsr >= 16U)
  {
    SEGGER_RTT_printf(0,
                      "Context    : External IRQ, IRQn = %u\r\n",
                      (unsigned int)(ipsr - 16U));
  }
  else
  {
    SEGGER_RTT_printf(0,
                      "Context    : System exception %u\r\n",
                      (unsigned int)ipsr);
  }

  SEGGER_RTT_printf(0,
                    "MSP        : 0x%08X\r\n",
                    (unsigned int)msp);

  SEGGER_RTT_printf(0,
                    "PSP        : 0x%08X\r\n",
                    (unsigned int)psp);

  SEGGER_RTT_printf(0,
                    "CONTROL    : 0x%08X\r\n",
                    (unsigned int)control);

  SEGGER_RTT_printf(0,
                    "PRIMASK    : 0x%08X\r\n",
                    (unsigned int)primask);

  SEGGER_RTT_printf(0,
                    "ICSR       : 0x%08X\r\n",
                    (unsigned int)icsr);

  SEGGER_RTT_printf(0,
                    "VECTACTIVE : %u\r\n",
                    (unsigned int)(icsr & 0x1FFU));

  SEGGER_RTT_printf(0,
                    "VECTPENDING: %u\r\n",
                    (unsigned int)((icsr >> 12U) & 0x1FFU));

  SEGGER_RTT_printf(0,
                    "SHCSR      : 0x%08X\r\n",
                    (unsigned int)SCB->SHCSR);

#if defined(__CORTEX_M) && (__CORTEX_M >= 3U) && (__CORTEX_M != 23U)

  {
    uint32_t cfsr = SCB->CFSR;

    SEGGER_RTT_WriteString(0,
                           "------------ Fault status ------------\r\n");

    SEGGER_RTT_printf(0,
                      "BASEPRI    : 0x%08X\r\n",
                      (unsigned int)__get_BASEPRI());

    SEGGER_RTT_printf(0,
                      "FAULTMASK  : 0x%08X\r\n",
                      (unsigned int)__get_FAULTMASK());

    SEGGER_RTT_printf(0,
                      "CFSR       : 0x%08X\r\n",
                      (unsigned int)cfsr);

    SEGGER_RTT_printf(0,
                      "MMFSR      : 0x%02X\r\n",
                      (unsigned int)(cfsr & 0xFFU));

    SEGGER_RTT_printf(0,
                      "BFSR       : 0x%02X\r\n",
                      (unsigned int)((cfsr >> 8U) & 0xFFU));

    SEGGER_RTT_printf(0,
                      "UFSR       : 0x%04X\r\n",
                      (unsigned int)((cfsr >> 16U) & 0xFFFFU));

    SEGGER_RTT_printf(0,
                      "HFSR       : 0x%08X\r\n",
                      (unsigned int)SCB->HFSR);

    SEGGER_RTT_printf(0,
                      "DFSR       : 0x%08X\r\n",
                      (unsigned int)SCB->DFSR);

    SEGGER_RTT_printf(0,
                      "AFSR       : 0x%08X\r\n",
                      (unsigned int)SCB->AFSR);

    if ((cfsr & (1UL << 7U)) != 0U)
    {
      SEGGER_RTT_printf(0,
                        "MMFAR      : 0x%08X valid\r\n",
                        (unsigned int)SCB->MMFAR);
    }
    else
    {
      SEGGER_RTT_WriteString(0,
                             "MMFAR      : invalid\r\n");
    }

    if ((cfsr & (1UL << 15U)) != 0U)
    {
      SEGGER_RTT_printf(0,
                        "BFAR       : 0x%08X valid\r\n",
                        (unsigned int)SCB->BFAR);
    }
    else
    {
      SEGGER_RTT_WriteString(0,
                             "BFAR       : invalid\r\n");
    }
  }

#endif

  SEGGER_RTT_WriteString(0,
                         "========================================\r\n"
                         "System halted\r\n"
                         "========================================\r\n");

  while (1)
  {
    __NOP();
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
  SEGGER_RTT_WriteString(0, "\r\n*** ASSERT FAILED *** ");
  SEGGER_RTT_WriteString(0, (const char *)file);
  SEGGER_RTT_printf(0, " Line=%lu\r\n", (unsigned long)line);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
