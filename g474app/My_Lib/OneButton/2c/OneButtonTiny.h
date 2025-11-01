// // -----
// // OneButtonTiny.h - Library for detecting button clicks, doubleclicks and long
// // press pattern on a single button. This class is implemented for use with the
// // Arduino environment. Copyright (c) by Matthias Hertel,
// // http://www.mathertel.de This work is licensed under a BSD style license. See
// // http://www.mathertel.de/License.aspx More information on:
// // http://www.mathertel.de/Arduino
// // -----
// // 01.12.2023 created from OneButtonTiny to support tiny environments.
// // -----

// #ifndef ONEBUTTONTINY_H
// #define ONEBUTTONTINY_H

// #include "main.h"

// // ----- Callback function types -----
// typedef void (*callbackFunction)(void);

// // 定义状态机的枚举类型
// typedef enum {
//   OCS_INIT = 0,
//   OCS_DOWN = 1,
//   OCS_UP = 2,
//   OCS_COUNT = 3,
//   OCS_PRESS = 6,
//   OCS_PRESSEND = 7,
// } stateMachine_t;

// // 定义 OneButtonTiny 的结构体
// typedef struct {
//   int _pin;
//   unsigned int _debounce_ms;
//   unsigned int _click_ms;
//   unsigned int _press_ms;
//   int _buttonPressed;
//   callbackFunction _clickFunc;
//   callbackFunction _doubleClickFunc;
//   callbackFunction _longPressStartFunc;

//   stateMachine_t _state;
//   int debouncedPinLevel;
//   int _lastDebouncePinLevel;
//   unsigned long _lastDebounceTime;
//   unsigned long now;
//   unsigned long _startTime;
//   int _nClicks;
// } OneButtonTiny;

// // ----- 函数声明 -----

// // 初始化 OneButtonTiny
// void OneButtonTiny_init(OneButtonTiny *button, int pin, int activeLow, int pullupActive);

// // 设置去抖时间
// void OneButtonTiny_setDebounceMs(OneButtonTiny *button, unsigned int ms);

// // 设置单击时间
// void OneButtonTiny_setClickMs(OneButtonTiny *button, unsigned int ms);

// // 设置长按时间
// void OneButtonTiny_setPressMs(OneButtonTiny *button, unsigned int ms);

// // 绑定单击事件
// void OneButtonTiny_attachClick(OneButtonTiny *button, callbackFunction newFunction);

// // 绑定双击事件
// void OneButtonTiny_attachDoubleClick(OneButtonTiny *button, callbackFunction newFunction);

// // 绑定长按事件
// void OneButtonTiny_attachLongPressStart(OneButtonTiny *button, callbackFunction newFunction);

// // 状态机处理
// void OneButtonTiny_tick(OneButtonTiny *button);
// void OneButtonTiny_tickWithLevel(OneButtonTiny *button, int level);

// // 重置按钮状态机
// void OneButtonTiny_reset(OneButtonTiny *button);

// // 检查是否空闲
// int OneButtonTiny_isIdle(const OneButtonTiny *button);

// #endif