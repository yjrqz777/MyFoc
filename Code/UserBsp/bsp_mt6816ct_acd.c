/***************************************************************************************************
* @版    权：深圳拓邦股份有限公司-微电研发中心-家电部
* @文 件 名：bsp_mt6816ct_acd.c
* @内容摘要：MT6816CT-ACD SPI 绝对式磁编码器 BSP 驱动
* @详细说明：本文件实现 MT6816CT-ACD 编码器底层驱动，包括：
*           1. 通过 SPI1 按 16 时钟协议读取内部寄存器
*           2. 合成 14 位原始角度并提取 No_Mag_Warning/PC 状态位
*           3. 提供机械角/电角度换算接口，供 FOC 快速环读取转子位置
* @当前版本：V1.0
* @作    者：家电部-软件组
* @完成日期：
* @记    录：
* @修改记录：
* @修改日期：
* @版 本 号：
* @修 改 人：
***************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bsp_mt6816ct_acd.h"
#include "spi.h"



static volatile tBspMt6816CtSampleDef tLastSample = {0u, 0u, 0u, 0u, 0u, 0u};

/**
 * @brief   角度归一化到 [0, 2π)
 * @param[in] f32Angle  待归一化角度(rad)
 * @return  归一化后的角度(rad)，范围 [0, 2π)
 */
static float BspMt6816CtNormalizeAngle(float f32Angle)
{
    while (f32Angle >= M_2PI)   /* 大于等于2π时循环减去2π */
    {
        f32Angle -= M_2PI;
    }
    while (f32Angle < 0.0f)                        /* 小于0时循环加上2π */
    {
        f32Angle += M_2PI;
    }

    return f32Angle;
}

/**
 * @brief   初始化 MT6816CT 编码器驱动
 * @note    释放片选(CSN 置高)并清零最近一次采样缓存；片选引脚由 MX_GPIO_Init()
 *          配置，本函数不初始化 SPI1
 */
void BspMt6816CtInit(void)
{
    /* CSN is active low.  The pin itself is configured by MX_GPIO_Init(). */
    HAL_GPIO_WritePin(BSP_MT6816CT_CSN_GPIO_PORT,
                      BSP_MT6816CT_CSN_PIN,
                      GPIO_PIN_SET);
    tLastSample.u8RegAngleHigh = 0u;   /* 清零角度高字节寄存器缓存 */
    tLastSample.u8RegAngleLow = 0u;    /* 清零角度低字节寄存器缓存 */
    tLastSample.u16RawAngle = 0u;      /* 清零原始角度 */
    tLastSample.u8NoMagWarning = 0u;   /* 清零无磁场告警标志 */
    tLastSample.u8Pc = 0u;             /* 清零奇偶校验位 */
    tLastSample.u8Valid = 0u;          /* 采样有效标志置0 */
}

/**
 * @brief   按 16 时钟 SPI 协议读取单个寄存器
 * @param[in]  u8Address  7 位寄存器地址
 * @param[out] pu8Data    寄存器数据输出指针(DO7..DO0)
 * @retval HAL_OK  MCU SPI 传输完成
 * @retval 其他    传输失败或参数非法
 * @note    首字节 bit7=1 表示读、bit6..0 为地址，第二字节为哑数据；MISO 在时钟
 *          9..16 期间返回数据。SPI 无从机应答，HAL_OK 仅代表 MCU 侧传输完成
 */
HAL_StatusTypeDef BspMt6816CtReadRegister(uint8_t u8Address,
                                             uint8_t *pu8Data)
{
    uint16_t u16TxFrame;
    uint16_t u16RxFrame = 0x0000u;
    HAL_StatusTypeDef eStatus;

    if (pu8Data == NULL)
    {
        return HAL_ERROR;   /* 输出指针为空直接返回错误 */
    }

    /* First byte: bit7=1 (read), bits6..0=A6..A0.  R/W=0 is a write
     * operation on MT6816CT.  The second byte is dummy data; MISO returns
     * DO7..DO0 during clocks 9..16. */
    u16TxFrame = (uint16_t)(((uint16_t)(0x80u | (u8Address & 0x7Fu))) << 8u);

    HAL_GPIO_WritePin(BSP_MT6816CT_CSN_GPIO_PORT,
                      BSP_MT6816CT_CSN_PIN,
                      GPIO_PIN_RESET);   /* 片选拉低，启动一次传输 */

    /* SPI1 uses 16-bit data size, therefore Size is one 16-bit frame. */
    eStatus = HAL_SPI_TransmitReceive(&hspi1,
                                      (uint8_t *)&u16TxFrame,
                                      (uint8_t *)&u16RxFrame,
                                      1u,
                                      BSP_MT6816CT_SPI_TIMEOUT_MS);

    HAL_GPIO_WritePin(BSP_MT6816CT_CSN_GPIO_PORT,
                      BSP_MT6816CT_CSN_PIN,
                      GPIO_PIN_SET);   /* 片选拉高，结束本次传输 */

    if (eStatus == HAL_OK)
    {
        *pu8Data = (uint8_t)u16RxFrame;   /* 取出低字节作为寄存器数据 */
    }

    return eStatus;
}

/**
 * @brief   读取并合成 14 位原始角度
 * @param[out] pu16RawAngle      原始角度计数输出指针(0~16383)
 * @param[out] pu8NoMagWarning   无磁场告警位输出指针(可为 NULL)
 * @param[out] pu8Pc             奇偶校验位输出指针(可为 NULL)
 * @retval HAL_OK  读取成功
 * @retval 其他    传输失败
 * @note    依次读取角度高/低字节寄存器，拼接为 14 位原始角度，并从低字节提取
 *          No_Mag_Warning 与 PC 状态位
 */
HAL_StatusTypeDef BspMt6816CtReadRawAngle(uint16_t *pu16RawAngle,
                                             uint8_t *pu8NoMagWarning,
                                             uint8_t *pu8Pc)
{
    uint8_t u8AngleHigh;
    uint8_t u8AngleLow;
    HAL_StatusTypeDef eStatus;

    if (pu16RawAngle == NULL)
    {
        return HAL_ERROR;   /* 输出指针为空直接返回错误 */
    }

    eStatus = BspMt6816CtReadRegister(BSP_MT6816CT_REG_ANGLE_HIGH,
                                         &u8AngleHigh);
    if (eStatus == HAL_OK)
    {
        eStatus = BspMt6816CtReadRegister(BSP_MT6816CT_REG_ANGLE_LOW,
                                             &u8AngleLow);
    }

    if (eStatus == HAL_OK)
    {
        *pu16RawAngle = (uint16_t)(((uint16_t)u8AngleHigh << 6u) |
                                   ((uint16_t)u8AngleLow >> 2u));   /* 高字节左移6位+低字节高6位 */
        if (pu8NoMagWarning != NULL)
        {
            *pu8NoMagWarning = (uint8_t)((u8AngleLow >> 1u) & 0x01u);   /* 提取No_Mag_Warning位 */
        }
        if (pu8Pc != NULL)
        {
            *pu8Pc = (uint8_t)(u8AngleLow & 0x01u);   /* 提取PC奇偶校验位 */
        }
    }

    return eStatus;
}

/**
 * @brief   读取编码器并更新最新采样缓存
 * @retval HAL_OK  本次采样有效并已缓存
 * @retval 其他    传输失败，采样标记为无效
 * @note    仅在单一上下文调用；闭环运行期间 SPI1 由电机快速环独占，前台通过
 *          BspMt6816CtGetLastSample 读取缓存
 */
HAL_StatusTypeDef BspMt6816CtUpdate(void)
{
    uint16_t u16RawAngle;
    uint8_t u8AngleHigh;
    uint8_t u8AngleLow;
    uint8_t u8NoMagWarning;
    uint8_t u8Pc;
    HAL_StatusTypeDef eStatus;

    eStatus = BspMt6816CtReadRegister(BSP_MT6816CT_REG_ANGLE_HIGH,
                                         &u8AngleHigh);
    if (eStatus == HAL_OK)
    {
        eStatus = BspMt6816CtReadRegister(BSP_MT6816CT_REG_ANGLE_LOW,
                                             &u8AngleLow);
    }
    if (eStatus == HAL_OK)
    {
        u16RawAngle = (uint16_t)(((uint16_t)u8AngleHigh << 6u) |
                                 ((uint16_t)u8AngleLow >> 2u));   /* 合成14位原始角度 */
        u8NoMagWarning = (uint8_t)((u8AngleLow >> 1u) & 0x01u);   /* 提取无磁场告警位 */
        u8Pc = (uint8_t)(u8AngleLow & 0x01u);                     /* 提取奇偶校验位 */
        tLastSample.u8RegAngleHigh = u8AngleHigh;   /* 缓存角度高字节 */
        tLastSample.u8RegAngleLow = u8AngleLow;     /* 缓存角度低字节 */
        tLastSample.u16RawAngle = u16RawAngle;      /* 缓存14位原始角度 */
        tLastSample.u8NoMagWarning = u8NoMagWarning;   /* 缓存无磁场告警 */
        tLastSample.u8Pc = u8Pc;                    /* 缓存奇偶校验位 */
        tLastSample.u8Valid = 1u;                   /* 标记本次采样有效 */
    }
    else
    {
        tLastSample.u8Valid = 0u;   /* 传输失败标记采样无效 */
    }

    if (tLastSample.u8Valid == 0u)
    {
        return 1; /* 标记本次采样有效 */
    }

    if (u8NoMagWarning == 1u)
    {
        return 1; /* 标记本次采样有效 */
    }

    return eStatus;
}

/**
 * @brief   查询最近一次编码器采样是否有效
 * @retval 1  采样有效
 * @retval 0  采样无效(未更新或上次传输失败)
 */
uint8_t BspMt6816CtIsSampleValid(void)
{
    return tLastSample.u8Valid;
}

/**
 * @brief   获取最近一次采样数据
 * @param[out] ptSample  采样数据结构体输出指针
 * @retval 1  拷贝成功
 * @retval 0  输出指针为空
 * @note    调用前应先通过 BspMt6816CtIsSampleValid 确认采样有效
 */
uint8_t BspMt6816CtGetLastSample(tBspMt6816CtSampleDef *ptSample)
{
    if (ptSample == NULL)
    {
        return 0u;   /* 指针非法返回0 */
    }

    ptSample->u8RegAngleHigh = tLastSample.u8RegAngleHigh;   /* 拷贝角度高字节 */
    ptSample->u8RegAngleLow = tLastSample.u8RegAngleLow;     /* 拷贝角度低字节 */
    ptSample->u16RawAngle = tLastSample.u16RawAngle;         /* 拷贝原始角度 */
    ptSample->u8NoMagWarning = tLastSample.u8NoMagWarning;   /* 拷贝无磁场告警 */
    ptSample->u8Pc = tLastSample.u8Pc;                       /* 拷贝奇偶校验位 */
    ptSample->u8Valid = tLastSample.u8Valid;                 /* 拷贝有效标志 */
    return 1u;
}

uint16_t u16BspMt6816CtGetRaw(void)
{
    return tLastSample.u16RawAngle;
}

/** @brief Return encoder raw counts in the motor-control clockwise-positive frame. */
uint16_t BspMt6816CtGetRawCw(void)
{
    return (uint16_t)(((BSP_MT6816CT_ACTIVE + 1u) - u16BspMt6816CtGetRaw()) & BSP_MT6816CT_ACTIVE);
}


/**
 * @brief   编码器计数转换为机械角
 * @param[in] u16RawAngle  编码器原始角度计数
 * @return  机械角度(rad)，范围 [0, 2π)
 * @note    按 14 位分辨率线性映射，先掩码取低 14 位
 */
float BspMt6816CtRaw2Angle(uint16_t u16RawAngle)
{
    u16RawAngle &= BSP_MT6816CT_ACTIVE;   /* 掩码取低14位 */
    return ((float)u16RawAngle * M_2PI) / (float)BSP_MT6816CT_ACTIVE;
}

/**
 * @brief   读取当前机械角度
 * @param[out] pf32MechanicalAngle  机械角度输出指针(rad)，范围 [0, 2π)
 * @retval HAL_OK  读取成功
 * @retval 其他    读取失败或参数非法
 */
HAL_StatusTypeDef BspMt6816CtReadMechanicalAngle(float *pf32MechanicalAngle)
{
    uint16_t u16RawAngle;
    HAL_StatusTypeDef eStatus;

    if (pf32MechanicalAngle == NULL)
    {
        return HAL_ERROR;   /* 输出指针为空直接返回错误 */
    }

    eStatus = BspMt6816CtReadRawAngle(&u16RawAngle, NULL, NULL);
    if (eStatus == HAL_OK)
    {
        *pf32MechanicalAngle = BspMt6816CtRaw2Angle(u16RawAngle);
    }

    return eStatus;
}

/**
 * @brief   编码器计数转换为 FOC 电角度
 * @param[in] u16RawAngle         编码器原始角度计数
 * @param[in] u8PolePairs         电机极对数，为 0 时直接返回 0
 * @param[in] f32ElectricalOffset 电气零位偏移(rad)
 * @return  电角度(rad)，范围 [0, 2π)
 * @note    机械角乘以极对数得到电角度，减去电气零位偏移后归一化到 [0, 2π)，
 *          供 FOC 坐标变换使用
 */
float BspMt6816CtToElecAngle(uint16_t u16RawAngle,uint8_t u8PolePairs,float elecAngleffset)
{
    float elecAngle;

    if (u8PolePairs == 0u)
    {
        return 0.0f;   /* 极对数为0时无法换算，返回0 */
    }

    elecAngle = BspMt6816CtRaw2Angle(u16RawAngle) *(float)u8PolePairs;   /* 机械角乘以极对数 */

    elecAngle -= elecAngleffset;   /* 减去电气零位偏移 */
    return BspMt6816CtNormalizeAngle(elecAngle);   /* 归一化到[0,2π) */
}
