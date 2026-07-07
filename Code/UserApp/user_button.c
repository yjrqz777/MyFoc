/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2026-01-06 20:19:37
 * LastEditTime: 2026-07-07 22:57:45
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description:
 * FilePath: \Myfoc2\Code\UserApp\user_button.c
 * @YJRQZ777
***************************************************************************************************/
#include "user_button.h"

#include "Task.h"
#include "bsp_button.h"
#include "SEGGER_RTT.h"

#define USER_BUTTON_NUM          4u
#define USER_BUTTON_ACTIVE_LEVEL 0u

static Button btn1;
static Button btn2;
static Button btn3;
static Button btn4;
static uint8_t button_inited = 0u;
static uint8_t button_last_event[USER_BUTTON_NUM] = {0u};

static Button *UserButton_GetHandle(uint8_t button_id)
{
    switch (button_id) {
    case 1:
        return &btn1;
    case 2:
        return &btn2;
    case 3:
        return &btn3;
    case 4:
        return &btn4;
    default:
        return 0;
    }
}

static uint8_t read_button_gpio(uint8_t button_id)
{
    return BspButton_ReadLevel(button_id);
}

static void button_event_handler(Button *btn)
{
    uint8_t index;
    ButtonEvent event;

    if (btn == 0 || btn->button_id == 0u || btn->button_id > USER_BUTTON_NUM) {
        return;
    }

    index = (uint8_t)(btn->button_id - 1u);
    event = button_get_event(btn);
    button_last_event[index] = (uint8_t)event;

    SEGGER_RTT_printf(0, "KEY%u event:%u repeat:%u\r\n",
                      btn->button_id,
                      (uint8_t)event,
                      button_get_repeat_count(btn));
}

static void button_attach_all_events(Button *btn)
{
    button_attach(btn, BTN_PRESS_DOWN, button_event_handler);
    button_attach(btn, BTN_PRESS_UP, button_event_handler);
    button_attach(btn, BTN_PRESS_REPEAT, button_event_handler);
    button_attach(btn, BTN_SINGLE_CLICK, button_event_handler);
    button_attach(btn, BTN_DOUBLE_CLICK, button_event_handler);
    button_attach(btn, BTN_LONG_PRESS_START, button_event_handler);
    button_attach(btn, BTN_LONG_PRESS_HOLD, button_event_handler);
}

void buttons_init(void)
{
    if (button_inited != 0u) {
        return;
    }

    button_init(&btn1, read_button_gpio, USER_BUTTON_ACTIVE_LEVEL, 1u);
    button_init(&btn2, read_button_gpio, USER_BUTTON_ACTIVE_LEVEL, 2u);
    button_init(&btn3, read_button_gpio, USER_BUTTON_ACTIVE_LEVEL, 3u);
    button_init(&btn4, read_button_gpio, USER_BUTTON_ACTIVE_LEVEL, 4u);

    button_attach_all_events(&btn1);
    button_attach_all_events(&btn2);
    button_attach_all_events(&btn3);
    button_attach_all_events(&btn4);

    button_start(&btn1);
    button_start(&btn2);
    button_start(&btn3);
    button_start(&btn4);

    button_inited = 1u;
}

uint8_t UserButton_GetRawMask(void)
{
    return BspButton_GetRawMask();
}

uint8_t UserButton_GetPressed(uint8_t button_id)
{
    Button *btn = UserButton_GetHandle(button_id);
    int pressed;

    if (btn == 0) {
        return 0u;
    }

    pressed = button_is_pressed(btn);
    return (pressed > 0) ? 1u : 0u;
}

uint8_t UserButton_GetPressedMask(void)
{
    uint8_t mask = 0u;

    if (UserButton_GetPressed(1u) != 0u) mask |= 0x01u;
    if (UserButton_GetPressed(2u) != 0u) mask |= 0x02u;
    if (UserButton_GetPressed(3u) != 0u) mask |= 0x04u;
    if (UserButton_GetPressed(4u) != 0u) mask |= 0x08u;

    return mask;
}

uint8_t UserButton_GetLastEvent(uint8_t button_id)
{
    if (button_id == 0u || button_id > USER_BUTTON_NUM) {
        return (uint8_t)BTN_NONE_PRESS;
    }

    return button_last_event[button_id - 1u];
}

uint16_t PtTaskButton(void)
{
    PT_BEGIN()
    {
        buttons_init();
    }

    while (1)
    {
        PT_WAIT_UNTIL(TICKS_INTERVAL / TIME_ms);
        button_ticks();
    }

    PT_END();
}
