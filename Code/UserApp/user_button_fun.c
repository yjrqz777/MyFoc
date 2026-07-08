
#include "user_button.h"
#include "bsp_button.h"
#include "user_system.h"
// static void button_event_handler(Button *btn)
// {
//     uint8_t index;
//     ButtonEvent event;

//     if (btn == 0 || btn->button_id == 0u || btn->button_id > USER_BUTTON_NUM) {
//         return;
//     }

//     index = (uint8_t)(btn->button_id - 1u);
//     event = button_get_event(btn);
//     button_last_event[index] = (uint8_t)event;

//     SEGGER_RTT_printf(0, "KEY%u event:%u repeat:%u\r\n",
//                       btn->button_id,
//                       (uint8_t)event,
//                       button_get_repeat_count(btn));
// }
void UserStatusSwitch(Button *btn)
{
    if (tSysData.enuState == E_SYS_STATE_POWER_ON)
    {
        tSysData.enuState = E_SYS_STATE_RUNNING;
        return;
    }

    if (tSysData.enuState == E_SYS_STATE_RUNNING)
    {
        tSysData.enuState = E_SYS_STATE_POWER_ON;
        return;
    }
}

//void UserStatusSwitch(Button *btn)
//{

//}