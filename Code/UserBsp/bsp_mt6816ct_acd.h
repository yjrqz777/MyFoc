/***************************************************************************************************
* @版    权：深圳拓邦股份有限公司-微电研发中心-家电部
* @文 件 名：bsp_mt6816ct_acd.h
* @内容摘要：MT6816CT-ACD SPI 绝对式磁编码器 BSP 驱动头文件
* @详细说明：本文件声明 MT6816CT-ACD 编码器驱动接口，包括：
*           1. 编码器硬件参数宏(分辨率、寄存器地址、SPI超时)
*           2. 采样数据结构体定义
*           3. 初始化、寄存器读取、角度换算等对外接口
*           硬件连接：SPI1，PD2/CSN、PB3/SCK、PB4/MISO、PB5/MOSI
* @当前版本：V1.0
* @作    者：家电部-软件组
* @完成日期：
* @记    录：
* @修改记录：
* @修改日期：
* @版 本 号：
* @修 改 人：
***************************************************************************************************/

#ifndef __BSP_MT6816CT_H__
#define __BSP_MT6816CT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "user_global.h"
/* CSN is PD2 in the CubeMX pinout.  Keep it local so generated main.h does
 * not need to be edited just to name one board-specific pin. */
#define BSP_MT6816CT_CSN_GPIO_PORT         (GPIOD)
#define BSP_MT6816CT_CSN_PIN               (GPIO_PIN_2)
/* MT6816CT SPI register addresses. */
#define BSP_MT6816CT_REG_ANGLE_HIGH        (0x03u) /* Angle[13:6] */
#define BSP_MT6816CT_REG_ANGLE_LOW         (0x04u) /* Angle[5:0], warning, PC */
#define BSP_MT6816CT_REG_SPEED_STATUS      (0x05u)

#define BSP_MT6816CT_ACTIVE      (MASK_RANGE(13,0)) /* 有效位 */
/* SPI access is called by the 20 kHz motor loop; fail within one system tick. */
#ifndef BSP_MT6816CT_SPI_TIMEOUT_MS
#define BSP_MT6816CT_SPI_TIMEOUT_MS        (1u)   /* SPI传输超时(ms) */
#endif

/** @brief Latest successfully read encoder sample, shared with foreground diagnostics. */
typedef struct tBspMt6816CtSampleDef
{
    uint8_t u8RegAngleHigh;      /* 角度高字节寄存器值(Angle[13:6]) */
    uint8_t u8RegAngleLow;       /* 角度低字节寄存器值(Angle[5:0]+状态位) */
    uint16_t u16RawAngle;        /* 14位原始角度计数(0~16383) */
    uint8_t u8NoMagWarning;      /* 无磁场告警位 1=磁场丢失 */
    uint8_t u8Pc;                /* 奇偶校验位 */
    uint8_t u8Valid;             /* 采样有效标志 1=有效 */
} tBspMt6816CtSampleDef;

/* 初始化 */
void BspMt6816CtInit(void);
/* 寄存器/角度读取 */
HAL_StatusTypeDef BspMt6816CtReadRegister(uint8_t u8Address, uint8_t *pu8Data);
HAL_StatusTypeDef BspMt6816CtReadRawAngle(uint16_t *pu16RawAngle, uint8_t *pu8NoMagWarning, uint8_t *pu8Pc);
HAL_StatusTypeDef BspMt6816CtUpdate(void);
uint8_t BspMt6816CtIsSampleValid(void);
uint8_t BspMt6816CtGetLastSample(tBspMt6816CtSampleDef *ptSample);
uint16_t u16BspMt6816CtGetRaw(void);
uint16_t BspMt6816CtGetRawCw(void);
HAL_StatusTypeDef BspMt6816CtReadMechanicalAngle(float *pf32MechanicalAngle);
/* 角度换算 */
float BspMt6816CtRaw2Angle(uint16_t u16RawAngle);
float BspMt6816CtToElecAngle(uint16_t u16RawAngle, uint8_t u8PolePairs,
                                float f32ElectricalOffset);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MT6816CT_H__ */
