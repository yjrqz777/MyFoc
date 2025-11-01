/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-05-25 22:01:30
 * LastEditTime: 2025-05-25 22:05:35
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \LittleTASK\LittleTask.c
 * @YJRQZ777
***************************************************************************************************/

// #include "board.h"
#include "LittleTask.h"

// 定义软件定时器数组
static software_timer timers[SOFT_TIMER_MAX];

 /**
  -  @brief  初始化单次定时器
  -  @note   None
  -  @param  u8rank: 要初始化的软件定时器，timeout: 超时时间
  -  @retval None
 */
void TASK_single_init(uint8_t u8rank, uint32_t timeout)
{
    TASK_interrupt_disable();
    if (u8rank < SOFT_TIMER_MAX)
    {
        timers[u8rank].counter = 0;
        timers[u8rank].timeout = timeout;
        timers[u8rank].is_timeout = 0;
        timers[u8rank].is_repeat = 0;
    }
    TASK_interrupt_enable();
}

/**
 -  @brief  初始化重复定时器
 -  @note   None
 -  @param  u8rank: 要初始化的软件定时器，timeout: 超时时间
 -  @retval None
*/
void TASK_repeat_init(uint8_t u8rank, uint32_t timeout)
{
    TASK_interrupt_disable();
    if (u8rank < SOFT_TIMER_MAX)
    {
        timers[u8rank].counter = 0;
        timers[u8rank].timeout = timeout;
        timers[u8rank].is_timeout = 0;
        timers[u8rank].is_repeat = 1;
    }
    TASK_interrupt_enable();
}

 /**
  -  @brief  停止一个软件定时器
  -  @note   None
  -  @param  timer：要停止的软件定时器
  -  @retval None
 */
void TASK_stop(uint8_t u8rank)
{
    TASK_interrupt_disable();
    if (u8rank < SOFT_TIMER_MAX)
    {
        timers[u8rank].is_timeout = 0;
        timers[u8rank].counter = 0;
        timers[u8rank].is_repeat = 0;
    }
    TASK_interrupt_enable();
}

 /**
  -  @brief  检查对应的定时器是否超时了
  -  @note   None
  -  @param  timer：要检查的定时器
  -  @retval None
 */
uint8_t TASK_is_timeout(uint8_t u8rank)
{
    uint8_t ret = 0;
    if (u8rank < SOFT_TIMER_MAX)
    {
        ret = timers[u8rank].is_timeout;
        if (ret)
        {
            timers[u8rank].is_timeout = 0;
        }
    }
    return ret;
}

/**
 -  @brief  复位对应的定时器
 -  @note   None
 -  @param  timer：要复位的定时器
 -  @retval None
*/
void TASK_reset(uint8_t u8rank)
{
    TASK_interrupt_disable();
    if (u8rank < SOFT_TIMER_MAX)
    {
        timers[u8rank].counter = 0;
        timers[u8rank].is_timeout = 0;
    }
    TASK_interrupt_enable();
}

 /**
  -  @brief  在GD32的SysTick_Handler中断服务程序中调用，每1ms调用一次，在这个工程里面他在
  board.c文件中被定时调用。
  -  @note   None
  -  @param  None
  -  @retval None
 */
void TASK_tick(void)
{
    for (uint8_t u8rank = SOFT_TIMER_0; u8rank < SOFT_TIMER_MAX; ++u8rank)
    {
        if (++timers[u8rank].counter >= timers[u8rank].timeout)
        {
            timers[u8rank].is_timeout = 1;
            if (timers[u8rank].is_repeat)
            {
                timers[u8rank].counter = 0;
            }
        }
    }
}

 /**
  -  @brief  在进行软件定时器数值修改时关闭全局中断来保护变量
  -  @note   None
  -  @param  None
  -  @retval None
 */
static void TASK_interrupt_disable(void)
{
    __disable_irq(); // 关闭全局中断
}

/**
 -  @brief  修改完对应软件定时器变量后恢复全局中段
 -  @note   None
 -  @param  None
 -  @retval None
*/
static void TASK_interrupt_enable(void)
{
    __enable_irq(); // 开启全局中断
}
