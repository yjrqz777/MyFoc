#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <stdint.h>

void buttons_init(void);
uint8_t UserButton_GetRawMask(void);
uint8_t UserButton_GetPressed(uint8_t button_id);
uint8_t UserButton_GetPressedMask(void);
uint8_t UserButton_GetLastEvent(uint8_t button_id);
uint16_t PtTaskButton(void);

#endif // USER_BUTTON_H
