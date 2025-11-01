/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-05-11 15:37:03
 * LastEditTime: 2025-10-08 18:40:28
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \g474app\My_Lib\FOC\foc.c
 * @YJRQZ777
***************************************************************************************************/

#include "foc.h"

Foc_DataDef Foc_Data = {0};
/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {ADC_HandleTypeDef} *hadc
***************************************************************************************************/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static uint32_t u32Ia = 0, u32Ib = 0, u32Ic = 0, u32Ibus = 0;
  static uint8_t u8count = 0;
    if (hadc->Instance == ADC1)
    {
        u8count++;
        u32Ia += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        u32Ib += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        u32Ic += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
        u32Ibus += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
        if (u8count >= 10)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // 点亮LED指示ADC转换完成
          Foc_Data.I_Def.Ia   = (uint32_t)(u32Ia * 1.0f / 10.0f); // 平均值
          Foc_Data.I_Def.Ib   = (uint32_t)(u32Ib * 1.0f / 10.0f); // 平均值
          Foc_Data.I_Def.Ic   = (uint32_t)(u32Ic * 1.0f / 10.0f); // 平均值
          Foc_Data.I_Def.Ibus = (uint32_t)(u32Ibus * 1.0f / 10.0f); // 平均值
          u32Ia = 0;
          u32Ib = 0;
          u32Ic = 0;
          u32Ibus = 0;
          u8count = 0; // 重置计数器
        }
    }
}

/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
***************************************************************************************************/
int PT_TASK_Test()
{
    PT_BEGIN()
    {
        // st7789v_init();
    }
    while (1)
    {
        PT_WAIT_UNTIL(10/TIME_ms); // 每10ms执行一次
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        // printf("%d,%d,%d,%d\n",
        // Foc_Data.I_Def.Ia,
        // Foc_Data.I_Def.Ib,
        // Foc_Data.I_Def.Ic,
        // Foc_Data.I_Def.Ibus
        // );
    }
    PT_END();
}




/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {ADC_HandleTypeDef} AdcNum
 * param {uint32_t} Channel
***************************************************************************************************/
uint16_t ADC_Read(ADC_HandleTypeDef AdcNum, uint32_t Channel)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t u16adcValue = 0;
    sConfig.Channel = Channel;
    // sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&AdcNum, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&AdcNum);
    HAL_ADC_PollForConversion(&AdcNum, HAL_MAX_DELAY);
    u16adcValue = (uint16_t)HAL_ADC_GetValue(&AdcNum);
    HAL_ADC_Stop(&AdcNum);
    return u16adcValue;
}
//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。
float voltage_power_supply=5.0;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
// 电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}


#include <math.h>

float my_fmod_basic(float x, float y) {
    // 使用标准算法，适合日常使用
    float div = x / y;
    float integer_part = (div > 0) ? floor(div) : ceil(div);
    return x - integer_part * y;
}







// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmodf(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}
// static float angle_remainder = 0.0f; 

// float _normalizeAngle(float angle){
//     angle += angle_remainder; // 补偿上次余数
//     float normalized = fmodf(angle, 2*PI);
//     angle_remainder = angle - normalized; // 记录超出部分
//     return normalized >= 0 ? normalized : (normalized + 2*PI);
// }

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) {

  // 计算占空比
  // 限制占空比从0到1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //写入PWM到PWM 0 1 2 通道
//   ledcWrite(0, dc_a*255);
//   ledcWrite(1, dc_b*255);
//   ledcWrite(2, dc_c*255);

    //   TIM1->CCR1 = dc_a*4250;
    //   TIM1->CCR2 = dc_b*4250;
    //   TIM1->CCR3 = dc_c*4250;

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dc_a*4250);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, dc_b*4250);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, dc_c*4250);
    //输出占空比
    // HAL_Delay(20);

    // printf("%f,%f,%f\n",dc_a,dc_b,dc_c);


}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 
// Ualpha = Uq * cos(angle_el);
// Ubeta = Uq * sin(angle_el);
  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}


//开环速度函数
float velocityOpenloop(float target_velocity){
  // printf("%d\n",111);
  // unsigned long now_us = HAL_GetTick()*1000;  //获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值
  // // printf("%d\n",now_us);
  // //计算当前每个Loop的运行时间间隔
  // float Ts = (now_us - open_loop_timestamp) * 1e-4f;

  // //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
  // if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  
  // Ts = 0.0001;
  // static float ftemp = 0.0f;
  // ftemp += 0.1f;
  // printf("%f,%f,%f,%f\n",shaft_angle,Ts,target_velocity,shaft_angle + target_velocity*Ts);
  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = _normalizeAngle(shaft_angle + 0.001f);
  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。
  // printf("%f,%f,%f,%f\n",shaft_angle,Ts,target_velocity,shaft_angle + target_velocity*Ts);
  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  float Uq = voltage_power_supply/3;
  // float Uq = voltage_power_supply/2 * (1 + 0.2f * fabsf(target_velocity)/5);
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, 7));
  
  // open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return Uq;
}
// float velocityOpenloop(float target_velocity) {
//     uint32_t now_us = HAL_GetTick();  // 获取毫秒并转换为微秒
//     float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    
//     if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
//     shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
//     float Uq = voltage_power_supply/5;
    
//     setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));
    
//     open_loop_timestamp = now_us;
//     return Uq;
// }
void FocInit(void)
{
    // velocityOpenloop(5);
}
/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
***************************************************************************************************/
int PT_TASK_FOC()
{
    PT_BEGIN()
    {
        // st7789v_init();
        printf("%d\n",222);
    }
    while (1)
    {
        PT_WAIT_UNTIL(10/TIME_ms); // 每100ms执行一次
        // velocityOpenloop(5);
        printf("%f,%f,%f\n",dc_a,dc_b,dc_c);
    }
    PT_END();
}

