/**
 * @file    user_motor.h
 * @brief   电机控制应用层接口
 * @note    提供电机初始化、启动、速度环、快速环、状态查询和在线调参接口
 */

#ifndef __USER_MOTOR_H__
#define __USER_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/* 控制周期与电机参数 */
#define USER_MOTOR_FAST_LOOP_HZ                   (20000u)
#define MOTOR_SPEED_LOOP_HZ                  (1000u)
#define MOTOR_POLE_PAIRS                     (7u)

/* 速度环与电流参考 */
#define USER_MOTOR_SPEED_REF_MAX                  (2200.0f)  /* 机械转速上限(rpm) */
#define USER_MOTOR_SPEED_REF_STEP                 (2.0f)     /* 速度参考斜坡步进(rpm/次，1kHz调用) */
#define USER_MOTOR_SPEED_STOP_THRESHOLD           (100.0f)    /* 停止判定阈值(rpm) */
#define USER_MOTOR_SPEED_ERROR_DEADBAND            (1.0f)    /* 速度PI误差死区(rpm) */
#define USER_MOTOR_IQ_REF_MAX                     (4.0f)     /* Iq参考上限(A) */
#define USER_MOTOR_IQ_REF_STEP                    (0.002f)   /* 20kHz快环Iq斜坡步进(A/次) */

/* 编码器测速与预定位 */
#define USER_MOTOR_ENCODER_SPEED_FILTER_ALPHA     (0.20f)
#define USER_MOTOR_ENC_WRAP_HALF_TURN             ((int32_t)(BSP_MT6816CT_ACTIVE / 2u))
#define USER_MOTOR_ENCODER_ALIGN_TIME_MS          (800u)
#define USER_MOTOR_ENCODER_ALIGN_TICKS            \
    ((USER_MOTOR_FAST_LOOP_HZ * USER_MOTOR_ENCODER_ALIGN_TIME_MS) / 1000u)
#define MOTOR_ENCODER_ALIGN_VOLTAGE          (6.0f)
#define MOTOR_ENCODER_ALIGN_STEP        (0.0002f)

/* 模拟输入与保护 */
#define USER_MOTOR_ADC_MID_SCALE                  (2048.0f)
#define USER_MOTOR_PHASE_CURRENT_LIMIT            (8.0f)
#define USER_MOTOR_OVC_DEBOUNCE_COUNT             (3u)

/** @brief 速度环观测变量(供 Keil Logic Analyzer 波形观察) */
typedef struct tUserMotorScopeDef
{
    float f32SpeedRef; /**< 速度参考(rpm) */
    float f32SpeedFbk; /**< 速度反馈(rpm) */
    float f32IqOutput; /**< 速度环 Iq 输出(A) */
} tUserMotorScopeDef;

void UsrMotorInit(void);
HAL_StatusTypeDef UsrMotorStart(void);
void UsrMotorFastLoop(void);
void UsrMotorSpeedLoop(void);
uint8_t UsrMotorGetStartupMode(void);
float UsrMotorGetSpeedRef(void);
float UsrMotorGetSpeed(void);
void UsrMotorSetSpeedTarget(float f32Target);
uint8_t UsrMotorIsOverCurrentFault(void);
float UsrMotorGetFaultIa(void);
float UsrMotorGetFaultIb(void);
float UsrMotorGetFaultIc(void);


/* Keil Logic Analyzer 中使用 g_tUserMotorScope.<成员名> 观察速度环。 */
extern volatile tUserMotorScopeDef g_tUserMotorScope;

#ifdef __cplusplus
}
#endif

#endif /* __USER_MOTOR_H__ */
