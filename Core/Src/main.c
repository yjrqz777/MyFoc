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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task.h"
#include <stdio.h>
#include "st7789v/st7789v.h"
#include <math.h>
#include "Userfoc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned int PT_TICK[TASK_MAX]={0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/***************************************************************************************************
 * 功能描述:
 * 输入参数:
 * 输出参数:
 * 返 回 值:
 * 其它说明:
 * param {int} ch
 * param {FILE} *f
 ***************************************************************************************************/
int fputc(int ch, FILE *f)
{
    uint8_t temp[1] = {ch};
    HAL_UART_Transmit(&huart3, temp, 1, 0xffff);
    // HAL_UART_Transmit_IT(&huart3, temp, 1);
    return ch;
}
/***************************************************************************************************
 * 功能描述:
 * 输入参数:
 * 输出参数:
 * 返 回 值:
 * 其它说明:
 * param {FILE} *f
 ***************************************************************************************************/
int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3, &ch, 1, 0xffff);
    return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

// ADC注入组采样值，依次为PA0, PA1, PA2, PA3
volatile uint16_t adc_injected[4] = {0};

// V/F控制参数
typedef struct {
    float freq;           // 输出频率 (Hz)
    float voltage;        // 输出电压 (0-100%)
    float vf_ratio;       // V/f比值 (V/Hz)
    float phase_acc;      // 相位累积器
    uint16_t pwm_max;     // PWM最大值
} VF_Control_t;

static VF_Control_t vf_ctrl = {
    .freq = 0.0f,
    .voltage = 0.0f,
    .vf_ratio = 10.0f,    // V/f = 10V/100Hz
    .phase_acc = 0.0f,
    .pwm_max = 5000       // TIM1 ARR = 5000
};


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*****************************************************************************
 * 功能描述: V/F控制 - 生成三相正弦波PWM
 * 参数说明:
 *   freq: 输出频率 (Hz)，范围 0-50Hz
 *   voltage: 输出电压 (%)，范围 0-100
 * 返回值: none
 * 说明: 在TIM6每100µs中断中调用，确保高频更新
 *****************************************************************************/
void VF_Control_Update(float freq, uint8_t voltage)
{
    uint16_t ccr1, ccr2, ccr3;
    float sin_u, sin_v, sin_w;
    float amplitude;
    float pi2 = 2.0f * 3.14159265f;
    float phase_step_val;
    
    // 限制输入范围
    if (freq < 0.0f) freq = 0.0f;
    if (freq > 50.0f) freq = 50.0f;
    if (voltage > 100) voltage = 100;
    
    vf_ctrl.freq = freq;
    vf_ctrl.voltage = voltage;
    
    // 相位累积（100µs更新一次，相位步长 = 2π * freq * 0.0001）
    phase_step_val = pi2 * freq * 0.0001f;  // 0.0001s = 100µs
    vf_ctrl.phase_acc += phase_step_val;
    if (vf_ctrl.phase_acc >= pi2) {
        vf_ctrl.phase_acc -= pi2;
    }
    
    // 计算三相正弦波（120度相位差）
    sin_u = sinf(vf_ctrl.phase_acc);
    sin_v = sinf(vf_ctrl.phase_acc - 2.0f * 3.14159265f / 3.0f);
    sin_w = sinf(vf_ctrl.phase_acc - 4.0f * 3.14159265f / 3.0f);
    
    // 使用用户输入的电压幅度
    amplitude = (voltage / 100.0f) * (vf_ctrl.pwm_max / 2.0f);
    
    // 计算CCR值（PWM占空比）
    // PWM中点 + 正弦波 * 幅度
    ccr1 = (uint16_t)(vf_ctrl.pwm_max / 2.0f + sin_u * amplitude);
    ccr2 = (uint16_t)(vf_ctrl.pwm_max / 2.0f + sin_v * amplitude);
    ccr3 = (uint16_t)(vf_ctrl.pwm_max / 2.0f + sin_w * amplitude);
    
    // 限制CCR范围
    if (ccr1 > vf_ctrl.pwm_max) ccr1 = vf_ctrl.pwm_max;
    if (ccr2 > vf_ctrl.pwm_max) ccr2 = vf_ctrl.pwm_max;
    if (ccr3 > vf_ctrl.pwm_max) ccr3 = vf_ctrl.pwm_max;
    
    // 更新PWM
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, ccr3);
}

static int PT_TASK_st7789()
{
  static uint8_t u8temp = 0;
    PT_BEGIN()
    {
        st7789v_init();
    }
    while (1)
    {
        PT_WAIT_UNTIL(100/TIME_ms); // 每100ms执行一次
        
        // FOC电流闭环已在TIM6中断中以100µs频率调用
        
        // 调试：打印关键信息
        DQCurrent_t dq = FOC_GetDQCurrent();
        uint8_t hall_a = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
        uint8_t hall_b = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
        uint8_t hall_c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        printf("ADC:%04d,%04d,%04d | Hall:%d%d%d | IdIq:%.2f,%.2f\r\n", 
               adc_injected[0], adc_injected[1], adc_injected[2],
               hall_a, hall_b, hall_c,
               dq.d, dq.q);
        
        // u8temp = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)<<1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)<<2;
        //LCD_ShowFloatNum1(0, 0, (ADC_Read(hadc2,ADC_CHANNEL_5) / 4096.0) * 3.3, 4, WHITE, BLACK, 24);
        LCD_ShowIntNum(0, 24, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6), 1, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 48, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7), 1, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 72, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0), 1, WHITE, BLACK, 24);
        
        // LCD显示ADC采样值（可选）
        LCD_ShowIntNum(0, 48, adc_injected[0], 4, WHITE, BLACK, 24); // PA0
        LCD_ShowIntNum(0, 72, adc_injected[1], 4, WHITE, BLACK, 24); // PA1
        LCD_ShowIntNum(0, 96, adc_injected[2], 4, WHITE, BLACK, 24); // PA2
        LCD_ShowIntNum(0, 120, adc_injected[3], 4, WHITE, BLACK, 24); // PA3
    }
    PT_END();
}
/* USER CODE END 0 */

// ADC注入组采样完成回调
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_injected[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); // PA0
        adc_injected[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); // PA1
        adc_injected[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3); // PA2
        adc_injected[3] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4); // PA3
    }
}

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
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
    // ADC校准和启动
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_InjectedStart_IT(&hadc1);  // 启动ADC1注入组中断
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim6); /* 此  100us 定时器用于 ADC触发  */
    HAL_TIM_Base_Start_IT(&htim7); /* 此  10ms  定时器用于  PT TASK */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    PT_TASK_REG(0, PT_TASK_st7789);
            printf("ADC: %04d,%04d,%04d,%04d\r\n", 
               adc_injected[0], adc_injected[1], adc_injected[2], adc_injected[3]);
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

// FOC控制参数
float foc_theta = 0.0f;     // 转子位置角（通过霍尔或编码器获取）
float foc_theta_step = 0.0f; // 相位步长

/*****************************************************************************
 * 功能描述: 根据霍尔传感器计算转子位置角
 * 说明: 6步霍尔编码，每步60度电角度
 *       HALL_A (PA6), HALL_B (PA7), HALL_C (PB0)
 * 返回值: 转子电角度（rad）
 *****************************************************************************/
float Get_Hall_Theta(void)
{
    // 临时不用霍尔传感器，使用固定斜坡角度
    // 每次中断增加角度，模拟转子旋转
    static float fake_theta = 0.0f;
    float pi = 3.14159265f;
    
    // 每次调用增加固定角度（100µs调用一次，一圈2π）
    // 如果要快速旋转，增加这个值
    fake_theta += 0.01f;  // 每100µs增加0.01弧度
    
    // 角度绕圈处理
    if (fake_theta > 2.0f * pi) {
        fake_theta -= 2.0f * pi;
    }
    
    return fake_theta;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // 100µs中断 - 开环V/F + FOC电流调节（混合策略）
        
        static float theta = 0.0f;
        float pi = 3.14159265f;
        
        // 每次增加角度（开环驱动）
        theta += 0.02f;
        if (theta > 2.0f * pi) {
            theta -= 2.0f * pi;
        }
        
        // === 步骤1：生成开环三相基础电压（确保电机能转） ===
        float U_base = 40.0f;
        float ua_base = U_base * sinf(theta);
        float ub_base = U_base * sinf(theta - 2.0f * pi / 3.0f);
        float uc_base = U_base * sinf(theta - 4.0f * pi / 3.0f);
        
        // === 步骤2：读ADC电流（用于FOC微调） ===
        float adcScale = 3.3f / 4096.0f / 20.0f / 0.1f;
        float ia = (float)(adc_injected[0] - 2048) * adcScale;
        float ib = (float)(adc_injected[1] - 2048) * adcScale;
        float ic = (float)(adc_injected[2] - 2048) * adcScale;
        
        // === 步骤3：FOC电流闭环（多不到20V修正） ===
        foc_theta = theta;
        FOC_CurrentLoop(ia, ib, ic, foc_theta);
        ThreePhaseVoltage_t uabc_foc = FOC_GetThreePhaseVoltage();
        
        // === 步骤4：基础电压 + FOC微调（0.3倍权重） ===
        float ua = ua_base + 0.3f * uabc_foc.ua;
        float ub = ub_base + 0.3f * uabc_foc.ub;
        float uc = uc_base + 0.3f * uabc_foc.uc;
        
        // 电压范围（-100~100）转换为PWM占空比（0~5000）
        uint16_t ccr1 = (uint16_t)(2500.0f + (ua / 100.0f) * 2500.0f);
        uint16_t ccr2 = (uint16_t)(2500.0f + (ub / 100.0f) * 2500.0f);
        uint16_t ccr3 = (uint16_t)(2500.0f + (uc / 100.0f) * 2500.0f);
        
        // PWM限幅
        if (ccr1 > 5000) ccr1 = 5000;
        if (ccr2 > 5000) ccr2 = 5000;
        if (ccr3 > 5000) ccr3 = 5000;
        if (ccr1 < 0) ccr1 = 0;
        if (ccr2 < 0) ccr2 = 0;
        if (ccr3 < 0) ccr3 = 0;
        
        // 更新PWM输出
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ccr1);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, ccr2);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, ccr3);
    }
    if (htim->Instance == TIM7)
    {
        // 10ms中断 - 任务调度
        TASK_TICK_UPDATE();
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
