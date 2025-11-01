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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task.h"
#include "vf_ctrl.h"
// #include "st7789v/st7789v.h"
#include "mt6816ct/mt6816.h"
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
    // HAL_UART_Transmit(&huart3, temp, 1, 0xffff);
    HAL_UART_Transmit_IT(&huart3, temp, 1);
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
/***************************************************************************************************
 * 功能描述:
 * 输入参数:
 * 输出参数:
 * 返 回 值:
 * 其它说明:
 * param {uint32_t} app_address
 ***************************************************************************************************/
void JumpToApp(uint32_t app_address)
{
    typedef void (*pFunction)(void);
    pFunction AppStart;

    __disable_irq();

    HAL_RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    uint32_t *app_sp = (uint32_t *)app_address;
    __set_MSP(*app_sp);
    SCB->VTOR = app_address;
    uint32_t *app_reset = (uint32_t *)(app_address + 4);
    AppStart = (pFunction)*app_reset;

    AppStart();

    while (1)
        ;
}
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int PT_TASK_st7789()
{
    PT_BEGIN()
    {
        st7789v_init();
    }
    while (1)
    {
        PT_WAIT_UNTIL(100/TIME_ms); // 每100ms执行一次
        LCD_ShowFloatNum1(0, 0, (ADC_Read(hadc2,ADC_CHANNEL_5) / 4096.0) * 3.3, 4, WHITE, BLACK, 24);
    }
    PT_END();
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
    __enable_irq(); // Enable global interrupts
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
	
	vf_ctrl_initialize();
	
	
	
    // HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    // HAL_ADCEx_Calibration_SetValue(&hadc1,ADC_SINGLE_ENDED,HAL_ADCEx_Calibration_GetValue(&hadc1,ADC_SINGLE_ENDED));
    

    // soft_timer_repeat_init(SOFT_TIMER_0, 100);
    // soft_timer_repeat_init(SOFT_TIMER_1, 1);
    // soft_timer_repeat_init(SOFT_TIMER_2, 300);

    // HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim6); /* 此  100us 定时器用于  */
    HAL_TIM_Base_Start_IT(&htim7); /* 此  10ms  定时器用于  PT TASK */

    // HAL_ADCEx_InjectedStart_IT(&hadc1);  

    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0.5*4250);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0.5*4250);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0.5*4250);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    printf("In APP !\n");
    uint16_t adc1_value[ADC1_NUM] = {0};
    uint16_t adc2_value[ADC2_NUM] = {0};
    //  ADC_ChannelConfTypeDef sConfig = {0};
    while (1)
    {
        PT_TASK_REG(0, PT_TASK_st7789);
        PT_TASK_REG(1, PT_TASK_FOC);

        PT_TASK_REG(2, PT_TASK_Test);
 
        // PT_TASK_REG(3, PT_TASK_mt6816);

        // printf("In APP 22!\n");
        // HAL_Delay(100);
        // LCD_ConvertAndSendDMA();
        // if (soft_timer_is_timeout(SOFT_TIMER_0))
        // {
        // 	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        //   LCD_ShowFloatNum1(0, 0, (ADC_Read(hadc1,ADC_CHANNEL_4) / 4096.0) * 3.3, 4, WHITE, BLACK, 24);
        //   // LCD_ConvertAndSendDMA();
        // }
        // TaskTest();
        // if (soft_timer_is_timeout(SOFT_TIMER_1))
        // {
        // FocLoop();
        // }
        // if (soft_timer_is_timeout(SOFT_TIMER_1))
        // {
        //   // LCD_Clear(WHITE);
        // }

        // if (soft_timer_is_timeout(SOFT_TIMER_2))
        // {
        //   // LCD_Clear(YELLOW);
        // }
        printf("%d,%d,%d\n",mcu_ccrx[0],mcu_ccrx[1],mcu_ccrx[2]);
        // printf("%d,%d,%d,%d,%d,%d,%d,%f,%d\n",adc1_value[0],adc1_value[1],adc1_value[2],adc1_value[3],adc1_value[4],adc1_value[5],adc1_value[6],(adc2_value[0]/4096.0)*3.3,adc2_value[1]);
        // LCD_ShowIntNum(0, 0, adc1_value[0], 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 24, adc1_value[1], 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 48, adc1_value[2], 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 72, adc1_value[3], 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 96, adc1_value[4], 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 120, adc1_value[5], 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(0, 144, adc1_value[6], 4, WHITE, BLACK, 24);
        // LCD_ShowChinese32x32(0, 0, "", WHITE, BLACK, 32, 0);

        // printf("%d,%d,%d\n",ADC_Read(hadc1,ADC_CHANNEL_1),ADC_Read(hadc1,ADC_CHANNEL_2),ADC_Read(hadc1,ADC_CHANNEL_3));

        // LCD_ShowFloatNum1(0, 0, (ADC_Read(hadc1,ADC_CHANNEL_4) / 4096.0) * 3.3, 4, WHITE, BLACK, 24);
        // LCD_ShowIntNum(120, 0, adc2_value[1], 4, WHITE, BLACK, 24);
        // LCD_ShowChineseTEST(0, 0, "FOC", WHITE, BLACK, 80,135, 0);
        // LCD_ShowIntNum(0, 192, adc2_value[1], 4, WHITE, BLACK, 24);
        //    JumpToApp(0x08000000);
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
    if (htim->Instance == TIM6)
    {
        // 处理 TIM6 的触发事件
//        velocityOpenloop(1);
			vf_ctrl_step();

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mcu_ccrx[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, mcu_ccrx[1]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, mcu_ccrx[2]);

    }
    if (htim->Instance == TIM7)
    {
        // 处理 TIM7 的触发事件
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
