#include "BspLcd.h"
#include "st7789v/st7789v.h"

void BspLcd_Init(void)
{
    st7789v_init();
}

void BspLcd_ShowMotorDebug(uint8_t hall_state, const uint16_t adc_raw[4])
{
    if (adc_raw == NULL)
    {
        return;
    }

    LCD_ShowIntNum(0, 24, hall_state, 1, WHITE, BLACK, 24);
    LCD_ShowIntNum(0, 48, adc_raw[0], 4, WHITE, BLACK, 24);
    LCD_ShowIntNum(0, 72, adc_raw[1], 4, WHITE, BLACK, 24);
    LCD_ShowIntNum(0, 96, adc_raw[2], 4, WHITE, BLACK, 24);
    LCD_ShowIntNum(0, 120, adc_raw[3], 4, WHITE, BLACK, 24);
}
