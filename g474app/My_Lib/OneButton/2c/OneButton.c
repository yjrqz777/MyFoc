#include "OneButton/2c/OneButton.h"

// 初始化 OneButton
void OneButton_init(OneButton *button, int pin, int activeLow, int pullupActive) {
  button->_pin = pin;
  button->_debounce_ms = 50;
  button->_click_ms = 400;
  button->_press_ms = 800;
  button->_idle_ms = 1000;
  button->_buttonPressed = activeLow ? LOW : HIGH;

  button->_pressFunc = NULL;
  button->_paramPressFunc = NULL;
  button->_pressFuncParam = NULL;

  button->_clickFunc = NULL;
  button->_paramClickFunc = NULL;
  button->_clickFuncParam = NULL;

  button->_doubleClickFunc = NULL;
  button->_paramDoubleClickFunc = NULL;
  button->_doubleClickFuncParam = NULL;

  button->_multiClickFunc = NULL;
  button->_paramMultiClickFunc = NULL;
  button->_multiClickFuncParam = NULL;

  button->_longPressStartFunc = NULL;
  button->_paramLongPressStartFunc = NULL;
  button->_longPressStartFuncParam = NULL;

  button->_longPressStopFunc = NULL;
  button->_paramLongPressStopFunc = NULL;
  button->_longPressStopFuncParam = NULL;

  button->_duringLongPressFunc = NULL;
  button->_paramDuringLongPressFunc = NULL;
  button->_duringLongPressFuncParam = NULL;

  button->_idleFunc = NULL;

  button->_state = OCS_INIT;
  button->_idleState = false;

  button->debouncedLevel = false;
  button->_lastDebounceLevel = false;
  button->_lastDebounceTime = 0;
  button->now = 0;

  button->_startTime = 0;
  button->_nClicks = 0;
  button->_maxClicks = 1;

  button->_long_press_interval_ms = 0;
  button->_lastDuringLongPressTime = 0;

  if (pullupActive) {
    pinMode(pin, INPUT_PULLUP);
  } else {
    pinMode(pin, INPUT);
  }
}

// 设置去抖时间
void OneButton_setDebounceMs(OneButton *button, int ms) {
  button->_debounce_ms = ms;
}

// 设置单击时间
void OneButton_setClickMs(OneButton *button, unsigned int ms) {
  button->_click_ms = ms;
}

// 设置长按时间
void OneButton_setPressMs(OneButton *button, unsigned int ms) {
  button->_press_ms = ms;
}

// 绑定单击事件
void OneButton_attachClick(OneButton *button, callbackFunction newFunction) {
  button->_clickFunc = newFunction;
}

// 绑定双击事件
void OneButton_attachDoubleClick(OneButton *button, callbackFunction newFunction) {
  button->_doubleClickFunc = newFunction;
}

// 绑定长按事件
void OneButton_attachLongPressStart(OneButton *button, callbackFunction newFunction) {
  button->_longPressStartFunc = newFunction;
}

// 状态机处理
void OneButton_tick(OneButton *button) {
  // TODO: 实现状态机逻辑
}

void OneButton_tickWithLevel(OneButton *button, int level) {
  // TODO: 实现状态机逻辑
}

// 重置按钮状态机
void OneButton_reset(OneButton *button) {
  button->_state = OCS_INIT;
}

// 检查是否空闲
int OneButton_isIdle(const OneButton *button) {
  return (button->_state == OCS_INIT);
}