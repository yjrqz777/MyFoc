
# 个人编码命名规范

## 概述
本文档定义了C语言编码命名规范，旨在统一代码风格，提高代码可读性和维护性。


## 命名规范

### 工程目录
| 项目 | 规范要求 | 示例 |
|------|----------|------|
| 用户程序目录 | 新建文件名称必须按照示例，第三方库/标准协议栈/客户提供的程序文件等除外 | `/UserApp`, `/UserBsp`, `/UserDrv` |
| 必需头文件 | 每个工程必须使用的头文件 | `User_global.h`, `user_config.h` |

### 文件名规范
| 类型 | 命名规则 | 示例 |
|------|----------|------|
| C/H文件 | 全小写+下划线 | `user_app.c/h`, `bsp_motor.c/h`, `drv_pwm.c/h` |

### 源文件与头文件组织

1. **C 文件中的头文件引用**：`#include` 应集中放在 C 文件前部，不应散落在变量定义、函数定义或其他实现代码之间。
2. **C 文件中的宏定义**：C 文件中应尽量避免使用 `#define`；需要被模块使用或复用的宏应统一定义在对应的 H 文件中。
3. **H 文件中的立即数宏**：使用宏定义整数、十六进制数、浮点数等立即数时，宏值必须使用圆括号包裹。
4. **H 文件保护符**：头文件保护符应由文件名转换为全大写下划线形式，并在首尾添加双下划线。例如 `user_time.h` 使用 `__USER_TIME_H__`。

```c
/* user_app.c */
#include "user_app.h"
#include "user_global.h"
```

```c
/* user_app.h */
#define FRAME_SOF       (0xAAu)
#define RETRY_COUNT     (3u)
#define SPEED_LIMIT     (1000.0f)
```

### 函数名规范
| 文件类型 | 命名规则 | 示例 |
|----------|----------|------|
| user_xxx.c/h | Usr+模块名称+功能名称 | `UsrSensorXxx()` |
| bsp_xxx.c/h | Bsp+模块名称+功能名称 | `BspMotorXxx()` |
| drv_xxx.c/h | Drv+模块名称+功能名称 | `DrvPwmXxx()` |

### 数据类型
- 使用 `user_global.h` 定义的标准类型
- **基本类型**: `uint8_t`, `int8_t`, `uint16_t`, `int16_t`, `uint32_t`, `int32_t`, `uint64_t`, `int64_t`, `float`, `double`

### 结构体命名
| 项目 | 命名规则 | 示例 |
|------|----------|------|
| 结构体定义 | t+大驼峰+Def | ```typedef struct tForExampleDef { }tForExampleDef;``` |
| 指针变量 | pt+大驼峰 | `tForExampleDef * ptTestDevice;` |
| 变量定义 | t+大驼峰 | `tForExampleDef tTestDevice;` |

### 联合体命名
| 项目 | 命名规则 | 示例 |
|------|----------|------|
| 联合体定义 | u+大驼峰+Def | ```typedef union uForExampleDef { }uForExampleDef;``` |
| 指针变量 | pu+大驼峰 | `uForExampleDef * puTestDevice;` |
| 变量定义 | u+大驼峰 | `uForExampleDef uTestDevice;` |

### 枚举类型命名
| 项目 | 命名规则 | 示例 |
|------|----------|------|
| 枚举定义 | e+大驼峰+Def | ```typedef enum { E_MODE_ID_NULL = 0, E_MODE_ID_POWERON, }eModeIdDef;``` |
| 枚举成员 | E+全大写+下划线 | `E_MODE_ID_NULL`, `E_MODE_ID_POWERON` |
| 枚举变量 | e+大驼峰 | `eModeIdDef eModeId;` |

### 宏定义命名
| 类型 | 命名规则 | 示例 |
|------|----------|------|
| 宏定义 | 全大写+下划线；立即数宏的值使用圆括号包裹 | `#define FRAME_SOF (0xAAu)`, `#define FRAME_EOF (0x55u)` |

### 变量命名
| 变量类型 | 命名规则 | 示例 |
|----------|----------|------|
| 全局变量 | 数据类型+大驼峰 | `uint32_t u32TotalCount`, `uint16_t u16TotalCount`, `uint8_t u8TotalCount` |
| 静态变量 | 数据类型+大驼峰 | `static uint32_t u32TotalCount`, `static uint16_t u16TotalCount` |
| 函数传入参数变量-非结构体指针变量 | p+数据类型缩写+大驼峰 | `uint8_t * pu8DataBuf`, `uint16_t * pu16DataBuf`, `uint32_t * pu32DataBuf` |
| 变量定义 | 数据类型+大驼峰 | `uint32_t u32TotalCount`, `uint16_t u16TotalCount` |
| 有符号变量 | 数据类型+大驼峰 | `int32_t s32TempValue`, `int16_t s16TempValue`, `int8_t s8TempValue` |
| 浮点变量 | 数据类型+大驼峰 | `float f32TempValue`, `double f64TempValue` |
| 指针变量 | p+数据类型+大驼峰 | `uint8_t * pu8DataBuf`, `uint16_t * pu16DataBuf` |
| 局部变量 | 大驼峰（无数据类型前缀） | `TotalAmount`, `Index`（允许使用 `i`, `j`, `k`） |

**数据类型缩写示例：**
- `uint32_t`: u32
- `uint16_t`: u16  
- `uint8_t`: u8
- `int32_t`: s32
- `int16_t`: s16
- `int8_t`: s8
- `float`: f32
- `double`: f64

## 使用示例

### 结构体定义示例
```c
typedef struct tUserFilterDef 
{
    uint16_t u16BufferSize;
    uint32_t u32SumValue;
} tUserFilterDef;

tUserFilterDef * ptUserFilter;  // 结构体指针
tUserFilterDef tUserFilter;      // 结构体变量
```

### 枚举定义示例
```c
typedef enum 
{
    E_FILTER_MODE_AVERAGE = 0,
    E_FILTER_MODE_MEDIAN,
    E_FILTER_MODE_SLIDING
} eFilterModeDef;

eFilterModeDef eCurrentMode;
```

### 函数定义示例
```c
// user_app.c 中的函数
void UsrFilterProcessData(void);

// bsp_motor.c 中的函数  
void BspMotorSetSpeed(uint16_t u16Speed);

// drv_pwm.c 中的函数
void DrvPwmSetDutyCycle(uint8_t u8Duty);
```

## 注意事项

1. **一致性**: 在整个项目中保持命名规范的一致性
2. **可读性**: 名称应清晰表达其用途
3. **模块化**: 按照模块功能进行命名区分
4. **避免冲突**: 防止局部变量与全局变量同名
