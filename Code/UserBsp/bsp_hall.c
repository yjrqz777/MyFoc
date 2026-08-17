/**
 * @file    bsp_hall.c
 * @brief   Hall sensor low-level driver and angle tracker.
 *******************************************************************************
 * @note    Read three Hall GPIO inputs, pack them into a 3-bit state, and track
 *          electrical angle/speed from the six valid Hall sectors.
 *******************************************************************************
 */

#include "bsp_hall.h"

uint8_t BspHallReadA(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
}

uint8_t BspHallReadB(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
}

uint8_t BspHallReadC(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);
}

tBspHallPinsDef BspHallReadPins(void)
{
    tBspHallPinsDef Pins;

    Pins.u8A = BspHallReadA();
    Pins.u8B = BspHallReadB();
    Pins.u8C = BspHallReadC();

    return Pins;
}

uint8_t BspHallGetState(void)
{
    uint8_t First = (uint8_t)(BspHallReadA() | (BspHallReadB() << 1) | (BspHallReadC() << 2));
    uint8_t Second = (uint8_t)(BspHallReadA() | (BspHallReadB() << 1) | (BspHallReadC() << 2));

    return (First == Second) ? First : 0u;
}

uint8_t BspHallIsValidState(uint8_t u8State)
{
    u8State &= 0x07u;
    return (u8State != 0x00u) && (u8State != 0x07u);
}

/* ===================================================================== *
 * Hall angle tracking with interpolation.
 * ===================================================================== */

#define HALL_PI                   (3.14159265f)
#define HALL_TWO_PI               (2.0f * HALL_PI)
#define HALL_SECTOR_ANGLE         (HALL_PI / 3.0f)      /* 60 electrical degrees. */
#define HALL_SPEED_FILTER_ALPHA   (0.20f)
#define HALL_PLL_KP               (0.15f)
#define HALL_SPEED_REPORT_SIGN    (-1.0f)               /* Report 154623 as negative physical speed. */
#define HALL_OFFSET_FILTER_ALPHA  (0.005f)
#define HALL_OFFSET_CALIB_SAMPLES (2000u)               /* 100 ms @ 20 kHz. */
#define HALL_STOP_TIMEOUT_MS      (500u)                /* No Hall edge for this time means stopped. */
#define HALL_STOP_TIMEOUT_TICKS   ((BSP_HALL_CONTROL_FREQ_HZ * HALL_STOP_TIMEOUT_MS) / 1000u)

/* Measured Hall chain when the open-loop electrical angle increases:
 * 1->5->4->6->2->3->1.
 * Keep this chain positive for the internal FOC electrical angle, otherwise
 * offset calibration compares opposite rotating angles and the current loop
 * drives away from the target.  BspHallGetElectricalSpeed() applies
 * HALL_SPEED_REPORT_SIGN so RTT still reports 154623 as negative speed. */
static const uint8_t u8ForwardNext[8] = {0u, 5u, 3u, 1u, 6u, 4u, 2u, 0u};

static volatile float f32Angle = 0.0f;
static volatile float f32Speed = 0.0f;
static volatile float f32AngleAtEdge = 0.0f;
static volatile uint32_t u32CycleCounter = 0u;
static volatile uint32_t u32LastEdgeCycle = 0u;
static volatile uint8_t u8LastState = 0u;
static volatile uint8_t u8TransitionCount = 0u;
static volatile uint8_t u8AngleValid = 0u;
static volatile uint8_t u8Stopped = 0u;
static volatile int8_t s8Direction = 0;

static volatile float f32Offset = 0.0f;
static volatile uint32_t u32OffsetSamples = 0u;
static volatile uint8_t u8OffsetDone = 0u;

void BspHallAngleInit(void)
{
    f32Angle = 0.0f;
    f32Speed = 0.0f;
    f32AngleAtEdge = 0.0f;
    u32CycleCounter = 0u;
    u32LastEdgeCycle = 0u;
    u8LastState = BspHallGetState();
    u8TransitionCount = 0u;
    u8AngleValid = 0u;
    u8Stopped = 0u;
    s8Direction = 0;
    f32Offset = 0.0f;
    u32OffsetSamples = 0u;
    u8OffsetDone = 0u;
}

void BspHallAngleUpdate(void)
{
    uint8_t State;
    uint32_t CycleDelta;
    float InstantSpeed;
    float DeltaTime;

    u32CycleCounter++;
    State = BspHallGetState();

    /* Ignore transient invalid Hall samples. Do not overwrite the last valid
     * state with 000/111 or a double-read mismatch around a switching edge. */
    if (BspHallIsValidState(State) != 0u)
    {
        if (BspHallIsValidState(u8LastState) == 0u)
        {
            u8LastState = State;
        }
        else if (State != u8LastState)
        {
            CycleDelta = u32CycleCounter - u32LastEdgeCycle;
            u8Stopped = 0u;

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
                /* Illegal jump. Resync to this valid state and wait for the next edge. */
                u8LastState = State;
                u32LastEdgeCycle = u32CycleCounter;
                f32Speed = 0.0f;
                u8TransitionCount = 0u;
                u8AngleValid = 0u;
                u8Stopped = 0u;
                return;
            }

            /* PLL-like edge correction: keep angle continuous and correct part
             * of the interpolation error at each Hall edge. */
            {
                float DeltaTimeAtEdge = (float)(u32CycleCounter - u32LastEdgeCycle) /
                                        (float)BSP_HALL_CONTROL_FREQ_HZ;
                float UnwrappedAngle = f32AngleAtEdge + f32Speed * DeltaTimeAtEdge;
                float TrueStep = (float)s8Direction * HALL_SECTOR_ANGLE;
                float InterpolatedStep = UnwrappedAngle - f32AngleAtEdge;
                float StepError = TrueStep - InterpolatedStep;

                f32AngleAtEdge = UnwrappedAngle + HALL_PLL_KP * StepError;
            }

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
            if (u8TransitionCount >= 2u)
            {
                u8AngleValid = 1u;
            }
            u8LastState = State;
        }
    }

    if ((u8AngleValid != 0u) &&
        ((u32CycleCounter - u32LastEdgeCycle) > HALL_STOP_TIMEOUT_TICKS))
    {
        /* Rotor has stopped or is too slow for interpolation.  Freeze the
         * angle and report zero speed, but keep the Hall angle valid.  Dropping
         * validity here makes low-speed control bounce between open-loop start
         * and Hall FOC whenever the 60-degree Hall edge interval is long. */
        f32Speed = 0.0f;
        f32AngleAtEdge = f32Angle;
        u8Stopped = 1u;
        s8Direction = 0;
    }

    if ((u8AngleValid != 0u) && (u8Stopped == 0u))
    {
        DeltaTime = (float)(u32CycleCounter - u32LastEdgeCycle) /
                    (float)BSP_HALL_CONTROL_FREQ_HZ;
        f32Angle = f32AngleAtEdge + f32Speed * DeltaTime;

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
    if (u8AngleValid == 0u)
    {
        return 0.0f;
    }

    return HALL_SPEED_REPORT_SIGN * f32Speed;
}

uint8_t BspHallIsAngleValid(void)
{
    return u8AngleValid;
}

void BspHallCalibrateOffsetUpdate(float f32ReferenceAngle)
{
    float Diff;

    if (u8OffsetDone != 0u)
    {
        return;
    }

    Diff = f32ReferenceAngle - f32Angle;
    while (Diff > HALL_PI)
    {
        Diff -= HALL_TWO_PI;
    }
    while (Diff < -HALL_PI)
    {
        Diff += HALL_TWO_PI;
    }

    if (u32OffsetSamples == 0u)
    {
        f32Offset = Diff;
    }
    else
    {
        f32Offset += HALL_OFFSET_FILTER_ALPHA * (Diff - f32Offset);
    }

    u32OffsetSamples++;
    if (u32OffsetSamples >= HALL_OFFSET_CALIB_SAMPLES)
    {
        u8OffsetDone = 1u;
    }
}

uint8_t BspHallIsOffsetCalibrated(void)
{
    return u8OffsetDone;
}

float BspHallGetOffset(void)
{
    return f32Offset;
}
