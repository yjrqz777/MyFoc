/**
 * @file    bsp_hall.c
 * @brief   Hall 传感器底层驱动实现
 *******************************************************************************
 * @note    通过 GPIO 读取 Hall 传感器三路电平，组合为 3-bit 状态码
 *          用于电机转子位置检测（六个有效扇区）
 *******************************************************************************
 */

#include "bsp_hall.h"

/**
 * @brief  读取 Hall A 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHallReadA(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
}

/**
 * @brief  读取 Hall B 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHallReadB(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
}

/**
 * @brief  读取 Hall C 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHallReadC(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);
}

/**
 * @brief  读取三个 Hall 引脚电平并打包为结构体
 * @return tBspHallPinsDef 结构体
 */
tBspHallPinsDef BspHallReadPins(void)
{
    tBspHallPinsDef Pins;

    Pins.u8A = BspHallReadA();
    Pins.u8B = BspHallReadB();
    Pins.u8C = BspHallReadC();

    return Pins;
}

/**
 * @brief  获取 Hall 状态码（组合三位 Hall 电平为 3-bit 值）
 * @return 状态码（0~7）
 * @note   bit0 = A 相, bit1 = B 相, bit2 = C 相
 *         有效扇区范围：1~6
 */
uint8_t BspHallGetState(void)
{
    tBspHallPinsDef Pins = BspHallReadPins();

    return (uint8_t)(Pins.u8A | (Pins.u8B << 1) | (Pins.u8C << 2));
}

/**
 * @brief  判断 Hall 状态是否有效
 * @param[in] u8State  Hall 状态码（0~7）
 * @retval 1  有效（状态在 1~6 之间）
 * @retval 0  无效（状态为 0 或 7，对应全部低或全部高）
 * @note   正常情况下 Hall 传感器不会输出 000 或 111
 */
uint8_t BspHallIsValidState(uint8_t u8State)
{
    u8State &= 0x07u;
    return (u8State != 0x00u) && (u8State != 0x07u);
}

/* ===================================================================== *
 *  Hall 角度跟踪（插值法）
 * ===================================================================== */

#define HALL_PI                 (3.14159265f)
#define HALL_TWO_PI             (2.0f * HALL_PI)
#define HALL_SECTOR_ANGLE       (HALL_PI / 3.0f)   /* 60° = π/3 rad */
#define HALL_SPEED_FILTER_ALPHA (0.15f)

/** @brief 正转时每个状态的后继状态：1→2→3→4→5→6→1 */
static const uint8_t u8ForwardNext[8] = {0u, 2u, 3u, 4u, 5u, 6u, 1u, 0u};

/* ---- 角度跟踪状态变量 ---- */
static volatile float f32Angle = 0.0f;          /**< 插值电角度 (rad) */
static volatile float f32Speed = 0.0f;          /**< 电角速度 (rad/s) */
static volatile float f32AngleAtEdge = 0.0f;    /**< 上次跳变时的角度基准 (rad) */
static volatile uint32_t u32CycleCounter = 0u;  /**< 快速环周期计数 */
static volatile uint32_t u32LastEdgeCycle = 0u;  /**< 上次跳变时的周期计数 */
static volatile uint8_t u8LastState = 0u;       /**< 上次 Hall 状态 */
static volatile uint8_t u8TransitionCount = 0u;  /**< 跳变次数（≥2 表示有效） */
static volatile int8_t s8Direction = 0;         /**< 方向：+1=正转, -1=反转, 0=未知 */

void BspHallAngleInit(void)
{
    f32Angle = 0.0f;
    f32Speed = 0.0f;
    f32AngleAtEdge = 0.0f;
    u32CycleCounter = 0u;
    u32LastEdgeCycle = 0u;
    u8LastState = BspHallGetState();
    u8TransitionCount = 0u;
    s8Direction = 0;
}

void BspHallAngleUpdate(void)
{
    uint8_t State;
    uint32_t CycleDelta;
    float InstantSpeed;
    float DeltaTime;

    u32CycleCounter++;
    State = BspHallGetState();

    if (State != u8LastState)
    {
        if (BspHallIsValidState(State) && BspHallIsValidState(u8LastState))
        {
            CycleDelta = u32CycleCounter - u32LastEdgeCycle;

            /* 方向检测 */
            if (u8ForwardNext[u8LastState] == State)
            {
                s8Direction = 1;
            }
            else if (u8ForwardNext[State] == u8LastState)
            {
                s8Direction = -1;
            }
            else
            {
                /* 非法跳变（跳过了一个扇区），忽略 */
                u8LastState = State;
                return;
            }

            /* 累加角度基准 */
            f32AngleAtEdge += (float)s8Direction * HALL_SECTOR_ANGLE;

            /* 测速 */
            if (CycleDelta > 0u)
            {
                InstantSpeed = HALL_SECTOR_ANGLE *
                               (float)BSP_HALL_CONTROL_FREQ_HZ /
                               (float)CycleDelta;
                InstantSpeed *= (float)s8Direction;

                if (u8TransitionCount == 0u)
                {
                    f32Speed = InstantSpeed;
                }
                else
                {
                    f32Speed += HALL_SPEED_FILTER_ALPHA * (InstantSpeed - f32Speed);
                }
            }

            u32LastEdgeCycle = u32CycleCounter;
            u8TransitionCount++;
        }
        u8LastState = State;
    }

    /* 角度插值 */
    if (u8TransitionCount >= 2u)
    {
        DeltaTime = (float)(u32CycleCounter - u32LastEdgeCycle) /
                    (float)BSP_HALL_CONTROL_FREQ_HZ;
        f32Angle = f32AngleAtEdge + f32Speed * DeltaTime;

        /* 归一化到 [0, 2π) */
        while (f32Angle >= HALL_TWO_PI)
        {
            f32Angle -= HALL_TWO_PI;
        }
        while (f32Angle < 0.0f)
        {
            f32Angle += HALL_TWO_PI;
        }
    }
}

float BspHallGetElectricalAngle(void)
{
    return f32Angle;
}

float BspHallGetElectricalSpeed(void)
{
    return f32Speed;
}

uint8_t BspHallIsAngleValid(void)
{
    return (u8TransitionCount >= 2u) ? 1u : 0u;
}
