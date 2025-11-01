
#ifndef __SOFT_TIMER_H__
#define __SOFT_TIMER_H__
#include "main.h"

#define SOFT_TIMER_MAX 10

// 定义软件定时器的结构体
// typedef struct
// {
//     volatile uint32_t counter;   // 计数器
//     volatile uint32_t timeout;   // 超时时间
//     volatile uint8_t is_timeout; // 超时标志
// } software_timer;


struct TaskParameterDef
{
    volatile uint32_t counter;   // 计数器
    volatile uint32_t timeout;   // 超时时间
    volatile uint8_t is_timeout; // 超时标志
} TaskParameter;



#define TASK_tick_update()  \
    do { \
        TaskParameter.counter ++; \
    } while(0)






void TASK_single_init(uint8_t timer, uint32_t timeout);
void TASK_repeat_init(uint8_t timer, uint32_t timeout);
void TASK_start(uint8_t timer);
void TASK_stop(uint8_t timer);
uint8_t TASK_is_timeout(uint8_t timer);
void TASK_reset(uint8_t timer);
void TASK_tick(void);
static void TASK_interrupt_disable(void);
static void TASK_interrupt_enable(void);

#endif // __SOFT_TIMER_H__
