#ifndef ONEBUTTON_H
#define ONEBUTTON_H

#include "main.h"

// ----- Callback function types -----
typedef void (*callbackFunction)(void);
typedef void (*parameterizedCallbackFunction)(void *);

// 定义状态机的枚举类型
typedef enum {
  OCS_INIT = 0,
  OCS_DOWN = 1,
  OCS_UP = 2,
  OCS_COUNT = 3,
  OCS_PRESS = 6,
  OCS_PRESSEND = 7,
} stateMachine_t;

// 定义 OneButton 的结构体
typedef struct {
  int _pin;
  int _debounce_ms;
  unsigned int _click_ms;
  unsigned int _press_ms;
  unsigned int _idle_ms;
  int _buttonPressed;

  callbackFunction _pressFunc;
  parameterizedCallbackFunction _paramPressFunc;
  void *_pressFuncParam;

  callbackFunction _clickFunc;
  parameterizedCallbackFunction _paramClickFunc;
  void *_clickFuncParam;

  callbackFunction _doubleClickFunc;
  parameterizedCallbackFunction _paramDoubleClickFunc;
  void *_doubleClickFuncParam;

  callbackFunction _multiClickFunc;
  parameterizedCallbackFunction _paramMultiClickFunc;
  void *_multiClickFuncParam;

  callbackFunction _longPressStartFunc;
  parameterizedCallbackFunction _paramLongPressStartFunc;
  void *_longPressStartFuncParam;

  callbackFunction _longPressStopFunc;
  parameterizedCallbackFunction _paramLongPressStopFunc;
  void *_longPressStopFuncParam;

  callbackFunction _duringLongPressFunc;
  parameterizedCallbackFunction _paramDuringLongPressFunc;
  void *_duringLongPressFuncParam;

  callbackFunction _idleFunc;

  stateMachine_t _state;
  bool _idleState;

  bool debouncedLevel;
  bool _lastDebounceLevel;
  unsigned long _lastDebounceTime;
  unsigned long now;

  unsigned long _startTime;
  int _nClicks;
  int _maxClicks;

  unsigned int _long_press_interval_ms;
  unsigned long _lastDuringLongPressTime;
} OneButton;

// ----- 函数声明 -----

// 初始化 OneButton
void OneButton_init(OneButton *button, int pin, int activeLow, int pullupActive);

// 设置去抖时间
void OneButton_setDebounceMs(OneButton *button, int ms);

// 设置单击时间
void OneButton_setClickMs(OneButton *button, unsigned int ms);

// 设置长按时间
void OneButton_setPressMs(OneButton *button, unsigned int ms);

// 绑定单击事件
void OneButton_attachClick(OneButton *button, callbackFunction newFunction);

// 绑定双击事件
void OneButton_attachDoubleClick(OneButton *button, callbackFunction newFunction);

// 绑定长按事件
void OneButton_attachLongPressStart(OneButton *button, callbackFunction newFunction);

// 状态机处理
void OneButton_tick(OneButton *button);
void OneButton_tickWithLevel(OneButton *button, int level);

// 重置按钮状态机
void OneButton_reset(OneButton *button);

// 检查是否空闲
int OneButton_isIdle(const OneButton *button);

#endif