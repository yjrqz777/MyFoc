/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-08-10 19:39:00
 * LastEditTime: 2025-10-08 18:16:23
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \g474app\My_Lib\mt6816ct\mt6816.c
 * @YJRQZ777
***************************************************************************************************/
#include "mt6816ct/mt6816.h"

#include "stdio.h"

MT6816_SPI_Signal_Typedef	mt6816_spi;

void REIN_MT6816_SPI_Signal_Init(void)
{
	mt6816_spi.sample_data = 0;
	mt6816_spi.angle = 0;
}

// 全局变量定义
volatile int32_t mt6816_prev_angle = 0;    // 上一次的角度值 (14位，0-16383)
volatile float mt6816_current_speed_rpm = 0.0f; // 计算出的速度值 (RPM)
volatile bool mt6816_first_sample = true;   // 首次采样标志，用于初始化

// 速度计算函数，每10ms调用一次
void RINE_MT6816_SPI_Get_SpeedData(void)
{
    uint16_t data_t[2];
    uint16_t data_r[2];
    int32_t current_angle_raw; // 当前原始角度值
    int32_t delta_angle;       // 角度差
    float speed_rpm;           // 临时速度变量

    // 1. 读取MT6816原始角度数据 (基于你提供的函数)
    data_t[0] = (0x80 | 0x03) << 8; // 0x8300
    data_t[1] = (0x80 | 0x04) << 8; // 0x8400

    MT6816_SPI_CS_L();
    HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
    MT6816_SPI_CS_H();
    MT6816_SPI_CS_L();
    HAL_SPI_TransmitReceive(&MT6816_SPI_Get_HSPI, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
    MT6816_SPI_CS_H();

    mt6816_spi.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
    current_angle_raw = mt6816_spi.sample_data >> 2; // 获取14位角度值

    // 2. 如果是第一次采样，只记录角度，不计算速度
    if (mt6816_first_sample) {
        mt6816_prev_angle = current_angle_raw;
        mt6816_first_sample = false;
        mt6816_current_speed_rpm = 0.0f;
        return;
    }

    // 3. 计算角度差，处理角度溢出（0-16383循环）
    delta_angle = current_angle_raw - mt6816_prev_angle;
    
    // 处理正向溢出：角度从16383附近跳到0附近
    if (delta_angle < -8192) {
        delta_angle += 16384;
    }
    // 处理负向溢出：角度从0附近跳到16383附近
    else if (delta_angle > 8192) {
        delta_angle -= 16384;
    }

    // 4. 计算转速 (M法测速原理)
    // 转速 (RPM) = [角度变化量 / 时间 (秒)] / (每圈角度数) * 60秒
    // 角度变化量: delta_angle (单位: count)
    // 时间: 0.01秒 (10ms)
    // 每圈角度数: 16384 counts/revolution
    speed_rpm = (delta_angle / 0.01f / 16384.0f) * 60.0f;

    // 5. (可选) 对速度进行低通滤波，使数值更平稳
    // 一阶低通滤波: filtered_speed = (1 - alpha) * old_speed + alpha * new_speed
    float filter_alpha = 0.2f; // 滤波系数，可调整 (值越小越平滑，响应越慢)
    mt6816_current_speed_rpm = (1.0f - filter_alpha) * mt6816_current_speed_rpm + filter_alpha * speed_rpm;

    // 6. 更新前一次角度值
    mt6816_prev_angle = current_angle_raw;

    // 7. (可选) 调试输出：角度(度), 原始速度, 滤波后速度
    // printf("Angle: %.2f°, Raw Speed: %.2f RPM, Filtered Speed: %.2f RPM\n", 
    //        360.0f * current_angle_raw / 16384.0f, 
    //        speed_rpm, 
    //        mt6816_current_speed_rpm);
    // printf("%.2f,%.2f,%.2f\n", 
    //        360.0f * current_angle_raw / 16384.0f, 
    //        speed_rpm, 
    //        mt6816_current_speed_rpm);
}

MT6816_Typedef	mt6816;

float REIN_MT6816_Get_AngleData()
{
	RINE_MT6816_SPI_Get_SpeedData();
	mt6816.angle_data = mt6816_spi.angle;   
}





/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
***************************************************************************************************/
int PT_TASK_mt6816()
{
    PT_BEGIN()
    {
        REIN_MT6816_SPI_Signal_Init();
    }
    while (1)
    {
        PT_WAIT_UNTIL(10/TIME_ms); // 每100ms执行一次
        RINE_MT6816_SPI_Get_SpeedData();
        // printf("%.2f\r\n", 360/4096*REIN_MT6816_Get_AngleData());
    }
    PT_END();
}








