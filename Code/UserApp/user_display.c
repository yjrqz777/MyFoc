#include "user_display.h"
#include "bsp_adc.h"
#include "bsp_hall.h"
#include "bsp_lcd.h"
#include "user_button.h"
#include "user_foc.h"
#include "SEGGER_RTT.h"
#include "Task.h"
#include "main.h"

#define USER_DISPLAY_PERIOD_MS 100u

void UserDisplay_Init(void)
{
    BspLcd_Init();
}

void UserDisplay_Update100ms(void)
{
    uint8_t hall_state = BspHall_GetState();
    DQCurrent_t dq = FOC_GetDQCurrent();
    (void)dq;

    BspLcd_ShowUInt(0, 24, hall_state, 1);
    BspLcd_ShowUInt(0, 48, BspAdc_GetInjectedRaw(0), 4);
    BspLcd_ShowUInt(0, 72, BspAdc_GetInjectedRaw(1), 4);
    BspLcd_ShowUInt(0, 96, BspAdc_GetInjectedRaw(2), 4);
    BspLcd_ShowUInt(0, 120, BspAdc_GetInjectedRaw(3), 4);

    BspLcd_ShowUInt(96, 24, UserButton_GetPressedMask(), 2);
    BspLcd_ShowUInt(96, 48, UserButton_GetPressed(1), 1);
    BspLcd_ShowUInt(96, 72, UserButton_GetPressed(2), 1);
    BspLcd_ShowUInt(96, 96, UserButton_GetPressed(3), 1);
    BspLcd_ShowUInt(96, 120, UserButton_GetPressed(4), 1);

    BspLcd_ShowUInt(144, 48, UserButton_GetLastEvent(1), 1);
    BspLcd_ShowUInt(144, 72, UserButton_GetLastEvent(2), 1);
    BspLcd_ShowUInt(144, 96, UserButton_GetLastEvent(3), 1);
    BspLcd_ShowUInt(144, 120, UserButton_GetLastEvent(4), 1);

    // SEGGER_RTT_printf(0, "Hall:%u IdIq:%.2f,%.2f\r\n", hall_state, dq.d, dq.q);
}


int PT_TASK_Display(void)
{
    PT_BEGIN()
    {
        UserDisplay_Init();
    }

    while (1)
    {
        PT_WAIT_UNTIL(100 / TIME_ms);
        UserDisplay_Update100ms();
    }

    PT_END();
}
