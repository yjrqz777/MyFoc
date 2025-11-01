// #include "OneButton/2c/OneButtonTiny.h"

// // 初始化 OneButtonTiny
// void OneButtonTiny_init(OneButtonTiny *button, int pin, int activeLow, int pullupActive) {
//   button->_pin = pin;
//   button->_debounce_ms = 50;
//   button->_click_ms = 400;
//   button->_press_ms = 800;
//   button->_buttonPressed = activeLow ? LOW : HIGH;
//   button->_clickFunc = NULL;
//   button->_doubleClickFunc = NULL;
//   button->_longPressStartFunc = NULL;
//   button->_state = OCS_INIT;
//   button->debouncedPinLevel = -1;
//   button->_lastDebouncePinLevel = -1;
//   button->_lastDebounceTime = 0;
//   button->now = 0;
//   button->_startTime = 0;
//   button->_nClicks = 0;

//   if (pullupActive) {
//     // pinMode(pin, INPUT_PULLUP);
//   } else {
//     // pinMode(pin, INPUT);
//   }
// }

// // 设置去抖时间
// void OneButtonTiny_setDebounceMs(OneButtonTiny *button, unsigned int ms) {
//   button->_debounce_ms = ms;
// }

// // 设置单击时间
// void OneButtonTiny_setClickMs(OneButtonTiny *button, unsigned int ms) {
//   button->_click_ms = ms;
// }

// // 设置长按时间
// void OneButtonTiny_setPressMs(OneButtonTiny *button, unsigned int ms) {
//   button->_press_ms = ms;
// }

// // 绑定单击事件
// void OneButtonTiny_attachClick(OneButtonTiny *button, callbackFunction newFunction) {
//   button->_clickFunc = newFunction;
// }

// // 绑定双击事件
// void OneButtonTiny_attachDoubleClick(OneButtonTiny *button, callbackFunction newFunction) {
//   button->_doubleClickFunc = newFunction;
// }

// // 绑定长按事件
// void OneButtonTiny_attachLongPressStart(OneButtonTiny *button, callbackFunction newFunction) {
//   button->_longPressStartFunc = newFunction;
// }

// // 状态机处理
// void OneButtonTiny_tick(OneButtonTiny *button) {
//   // TODO: 实现状态机逻辑
// }

// void OneButtonTiny_tickWithLevel(OneButtonTiny *button, int level) {
//   // TODO: 实现状态机逻辑
// }

// // 重置按钮状态机
// void OneButtonTiny_reset(OneButtonTiny *button) {
//   button->_state = OCS_INIT;
// }

// // 检查是否空闲
// int OneButtonTiny_isIdle(const OneButtonTiny *button) {
//   return (button->_state == OCS_INIT);
// }