#include "bsp_lcd.h"
#include "st7789v/st7789v.h"

void BspLcd_Init(void)
{
    st7789v_init();
}

void BspLcd_ShowUInt(uint16_t x, uint16_t y, uint32_t value, uint8_t len)
{
    LCD_ShowIntNum(x, y, value, len, WHITE, BLACK, 24);
}
