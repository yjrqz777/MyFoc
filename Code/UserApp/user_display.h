#ifndef __USER_DISPLAY_H__
#define __USER_DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

void UserDisplay_Init(void);
void UserDisplay_Update100ms(void);
void UserDisplay_Poll(void);
int PT_TASK_Display(void);
#ifdef __cplusplus
}
#endif

#endif
