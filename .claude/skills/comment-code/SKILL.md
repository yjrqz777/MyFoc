---
name: comment-code
description: Add or improve Doxygen-style Chinese comments in C/H source files. Comment functions, macros, structs, and file headers using @brief/@param/@return/@retval/@note format.
---

# /comment-code — 为 C/H 源文件添加中文 Doxygen 注释

为当前项目中的 `.c` / `.h` 源文件添加或补充 **Doxygen 风格中文注释**。

## 注释格式

### 文件头注释

每个 `.c` / `.h` 文件顶部应当有：

```c
/**
 * @file    filename.c
 * @brief   一句话描述文件功能
 *******************************************************************************
 * @note    关键设计说明、注意事项
 *******************************************************************************
 */
```

### 函数注释

每个函数（包括 `static` 函数）前应当有：

```c
/**
 * @brief  功能描述（一句话，中文）
 * @param[in]  param1  参数说明
 * @param[out] param2  输出参数说明
 * @retval 0    含义
 * @retval 1    含义
 * @return 返回值说明（没有 @retval 时用）
 * @note   注意事项、调用约束、频率要求等
 */
```

### 宏定义注释

```c
/** @brief 简要说明 */
#define FOO 42

/**
 * @brief  复杂宏的说明
 * @param[in] x  参数含义
 */
#define FOO(x)  do { ... } while(0)
```

### 结构体/枚举注释

```c
/** @brief 结构体功能说明 */
typedef struct {
    uint8_t a;  /**< 成员 a 说明 */
    uint8_t b;  /**< 成员 b 说明 */
} MyStruct_t;
```

## 注释原则（重要）

1. **注释在 .c 文件中，不在 .h 中重复** — `.h` 文件仅保留文件头注释 + 宏/结构体注释，函数声明不写 Doxygen 块
2. `.c` 文件每个函数都必须有完整的 Doxygen 注释
3. **以代码逻辑为准** — 必须阅读并理解函数实际实现后写注释。如果已有旧注释与代码行为不符，**以代码逻辑为准重写注释**，不要照搬旧注释
4. 中文描述，简洁准确
5. `@note` 标注调用频率、硬件约束、前置条件等关键信息
6. 不注释第三方库代码（SEGGER RTT、CMSIS Driver、HAL 库等）
7. 不注释纯数据文件（font.h、pic.h 等）
8. 不修改代码逻辑，只添加/修改注释

## 用法示例

```bash
# 为所有 .c 文件补充注释
# (在对话中直接说 "注释 Code 文件夹的代码")
```
