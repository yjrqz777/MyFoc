/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2026-03-19
 * Description: FOC电流闭环控制实现
 * FilePath: \Myfoc2\Code\UserApp\user_foc.c
 * @YJRQZ777
***************************************************************************************************/

#include "user_foc.h"
#include <math.h>

/***************************************************************************************************
 * 数据结构定义
***************************************************************************************************/

// 两相电流/电压（α-β坐标系）
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

// dq轴电流（旋转坐标系）
typedef struct {
    float d;    // 直轴
    float q;    // 交轴
} DQ_t;

// PI调节器参数
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float integral;     // 积分累积
    float output;       // 输出
    float max_output;   // 输出限幅
} PI_Controller_t;

// FOC电流环控制结构体
typedef struct {
    float ia, ib, ic;               // 三相电流采样值
    AlphaBeta_t iAlphaBeta;         // α-β坐标系电流
    DQ_t iDQ_Real;                  // 实际d-q轴电流
    DQ_t iDQ_Ref;                   // 目标d-q轴电流
    DQ_t uDQ;                       // d-q轴输出电压
    AlphaBeta_t uAlphaBeta;         // α-β输出电压
    
    PI_Controller_t pi_d;           // d轴PI控制器
    PI_Controller_t pi_q;           // q轴PI控制器
    
    float theta;                    // 转子位置角（弧度）
    float sin_theta;
    float cos_theta;
    float ts;                       // 控制周期（50µs = 0.00005s）
} FOC_Current_Loop_t;

#define FOC_CONTROL_TS      0.00005f
#define FOC_VOLTAGE_LIMIT   80.0f

// 全局FOC变量
static FOC_Current_Loop_t foc = {
    .iDQ_Ref = {0.0f, 0.0f},       // Id=0，Iq由电机模块斜坡给定
    .theta = 0.0f,
    .ts = FOC_CONTROL_TS,          // 50µs
    .pi_d = {
        .kp = 0.35f,               // 0.35
        .ki = 2.0f,                // 2.0
        .integral = 0.0f,
        .output = 0.0f,
        .max_output = 100.0f       
    },
    .pi_q = {
        .kp = 0.5f,                // 0.2 - 保守增益
        .ki = 1.5f,               // 0.02
        .integral = 0.0f,
        .output = 0.0f,
        .max_output = 100.0f       
    }
};

/***************************************************************************************************
 * 功能描述: Clarke变换 - 三相(a,b,c)转两相(α,β)
 * 输入参数: ia, ib, ic - 三相电流
 * 输出参数: 返回α-β坐标系电流
 * 返回值: AlphaBeta_t
 * 说明: 克拉克变换矩阵 [ α ]   [ 1   -1/2  -1/2 ] [ ia ]
 *                    [ β ] = [ 0  √3/2  -√3/2] [ ib ]
 *                                                [ ic ]
***************************************************************************************************/
AlphaBeta_t Clarke_Transform(float ia, float ib, float ic)
{
    AlphaBeta_t iab;
    
    // α = ia
    iab.alpha = ia;
    
    // β = (√3/3) * (ib - ic) = (1/√3) * (ib - ic)
    // 实际实现：β = (2/3) * ib - (1/3) * ia - (1/3) * ic
    iab.beta = 0.577350f * (ib - ic);  // 1/sqrt(3)
    
    return iab;
}

/***************************************************************************************************
 * 功能描述: Park变换 - (α,β)转dq坐标系
 * 输入参数: 
 *   iAlphaBeta - α-β坐标系电流
 *   theta - 转子位置角（弧度）
 *   sin_theta, cos_theta - 角度的sin和cos
 * 输出参数: 返回d-q轴电流
 * 返回值: DQ_t
 * 说明: [ id ]   [ cos(θ)  sin(θ) ] [ iα ]
 *       [ iq ] = [-sin(θ)  cos(θ) ] [ iβ ]
***************************************************************************************************/
DQ_t Park_Transform(AlphaBeta_t iab, float sin_theta, float cos_theta)
{
    DQ_t idq;
    
    idq.d = cos_theta * iab.alpha + sin_theta * iab.beta;
    idq.q = -sin_theta * iab.alpha + cos_theta * iab.beta;
    
    return idq;
}

/***************************************************************************************************
 * 功能描述: 反Park变换 - dq坐标系转(α,β)
 * 输入参数:
 *   udq - d-q轴电压
 *   sin_theta, cos_theta - 角度的sin和cos
 * 输出参数: 返回α-β坐标系电压
 * 返回值: AlphaBeta_t
 * 说明: [ uα ]   [ cos(θ)  -sin(θ) ] [ ud ]
 *       [ uβ ] = [ sin(θ)   cos(θ) ] [ uq ]
***************************************************************************************************/
AlphaBeta_t InversePark_Transform(DQ_t udq, float sin_theta, float cos_theta)
{
    AlphaBeta_t uab;
    
    uab.alpha = cos_theta * udq.d - sin_theta * udq.q;
    uab.beta = sin_theta * udq.d + cos_theta * udq.q;
    
    return uab;
}

/***************************************************************************************************
 * 功能描述: 反Clarke变换 - (α,β)转三相
 * 输入参数: uab - α-β坐标系电压
 * 输出参数: 返回三相电压
 * 返回值: ThreePhaseVoltage_t
 * 说明: [ ua ]   [ 1    0   ] 
 *       [ ub ] = [-1/2  √3/2] [ uα ]
 *       [ uc ]   [-1/2 -√3/2] [ uβ ]
 * 满足: ua + ub + uc = 0（中性点）
***************************************************************************************************/
ThreePhaseVoltage_t InverseClarke_Transform(AlphaBeta_t uab)
{
    ThreePhaseVoltage_t uabc;
    
    uabc.ua = uab.alpha;
    uabc.ub = -0.5f * uab.alpha + 0.866025f * uab.beta;
    uabc.uc = -0.5f * uab.alpha - 0.866025f * uab.beta;
    
    return uabc;
}

/***************************************************************************************************
 * 功能描述: PI控制器
 * 输入参数:
 *   pi - PI控制器参数
 *   error - 当前误差
 *   ts - 控制周期
 * 输出参数: none
 * 返回值: none
 * 公式: u(n) = kp*e(n) + ki*T*Σe(n)
***************************************************************************************************/
void PI_Controller_Update(PI_Controller_t *pi, float error, float ts)
{
    // 比例项
    float p_term = pi->kp * error;
    
    // 积分项
    pi->integral += error * ts;
    float i_term = pi->ki * pi->integral;
    
    // 总输出
    pi->output = p_term + i_term;
    
    // 输出限幅（饱和）
    if (pi->output > pi->max_output) {
        pi->output = pi->max_output;
        pi->integral -= error * ts;  // 反饱和
    }
    if (pi->output < -pi->max_output) {
        pi->output = -pi->max_output;
        pi->integral -= error * ts;  // 反饱和
    }
}

static void FOC_LimitDQVoltage(DQ_t *udq, float max_voltage)
{
    float magnitude_sq = (udq->d * udq->d) + (udq->q * udq->q);
    float max_sq = max_voltage * max_voltage;

    if (magnitude_sq > max_sq)
    {
        float scale = max_voltage / sqrtf(magnitude_sq);
        udq->d *= scale;
        udq->q *= scale;
    }
}

void FOC_Reset(void)
{
    foc.iDQ_Ref.d = 0.0f;
    foc.iDQ_Ref.q = 0.0f;
    foc.iDQ_Real.d = 0.0f;
    foc.iDQ_Real.q = 0.0f;
    foc.uDQ.d = 0.0f;
    foc.uDQ.q = 0.0f;
    foc.uAlphaBeta.alpha = 0.0f;
    foc.uAlphaBeta.beta = 0.0f;
    foc.pi_d.integral = 0.0f;
    foc.pi_d.output = 0.0f;
    foc.pi_q.integral = 0.0f;
    foc.pi_q.output = 0.0f;
}

/***************************************************************************************************
 * 功能描述: FOC电流闭环主控制函数
 * 输入参数: 
 *   ia, ib, ic - 采样的三相电流（A）
 *   theta - 转子位置角（弧度）
 *   id_ref, iq_ref - 目标d-q轴电流
 * 输出参数: pwm1, pwm2, pwm3 - 三相PWM占空比
 * 返回值: none
 * 调用频率: 20kHz（50µs）
***************************************************************************************************/
void FOC_CurrentLoop(float ia, float ib, float ic, float theta)
{
    // === 步骤1：Clarke变换 (a,b,c) → (α,β) ===
    foc.ia = ia;
    foc.ib = ib;
    foc.ic = ic;
    foc.iAlphaBeta = Clarke_Transform(ia, ib, ic);
    
    // === 步骤2：计算角度的三角函数 ===
    foc.theta = theta;
    foc.sin_theta = sinf(theta);
    foc.cos_theta = cosf(theta);
    
    // === 步骤3：Park变换 (α,β) → (d,q) ===
    foc.iDQ_Real = Park_Transform(foc.iAlphaBeta, foc.sin_theta, foc.cos_theta);
    
    // === 步骤4：电流误差计算 ===
    float error_d = foc.iDQ_Ref.d - foc.iDQ_Real.d;
    float error_q = foc.iDQ_Ref.q - foc.iDQ_Real.q;
    
    // === 步骤5：PI控制器调节 ===
    PI_Controller_Update(&foc.pi_d, error_d, foc.ts);
    PI_Controller_Update(&foc.pi_q, error_q, foc.ts);
    
    foc.uDQ.d = foc.pi_d.output;
    foc.uDQ.q = -foc.pi_q.output;
    FOC_LimitDQVoltage(&foc.uDQ, FOC_VOLTAGE_LIMIT);
    
    // === 步骤6：反Park变换 (d,q) → (α,β) ===
    foc.uAlphaBeta = InversePark_Transform(foc.uDQ, foc.sin_theta, foc.cos_theta);
    
    // === 步骤7：反Clarke变换 (α,β) → (a,b,c) ===
    // 此时foc.uAlphaBeta已包含闭环调节的三相电压
}

/***************************************************************************************************
 * 功能描述: 获取FOC电流环的输出（三相电压）
 * 输入参数: none
 * 输出参数: 返回三相电压（-100 ~ 100）
 * 返回值: ThreePhaseVoltage_t
***************************************************************************************************/
ThreePhaseVoltage_t FOC_GetThreePhaseVoltage(void)
{
    ThreePhaseVoltage_t uabc = InverseClarke_Transform(foc.uAlphaBeta);
    return uabc;
}

/***************************************************************************************************
 * 功能描述: 设置FOC目标电流
 * 输入参数: 
 *   id_ref - 目标直轴电流（A），通常设为0
 *   iq_ref - 目标交轴电流（A），控制输出转矩
 * 输出参数: none
 * 返回值: none
***************************************************************************************************/
void FOC_SetCurrentReference(float id_ref, float iq_ref)
{
    foc.iDQ_Ref.d = id_ref;
    foc.iDQ_Ref.q = iq_ref;
}

/***************************************************************************************************
 * 功能描述: 获取FOC实际电流（dq轴）
 * 输入参数: none
 * 输出参数: none
 * 返回值: DQCurrent_t
***************************************************************************************************/
DQCurrent_t FOC_GetDQCurrent(void)
{
    DQCurrent_t dq;
    dq.d = foc.iDQ_Real.d;
    dq.q = foc.iDQ_Real.q;
    return dq;
}

/***************************************************************************************************
 * 功能描述: 获取FOC控制参数（调试用）
 * 输入参数: none
 * 输出参数: none
 * 返回值: FOC_Current_Loop_t
***************************************************************************************************/
FOC_Current_Loop_t* FOC_GetParameters(void)
{
    return &foc;
}
