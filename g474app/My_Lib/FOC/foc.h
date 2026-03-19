#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#define PI 3.1415926535


typedef struct Foc_DataDef
{
    struct
    {
        uint32_t Ia;  // A相电流
        uint32_t Ib;  // B相电流
        uint32_t Ic;  // C相电流
        uint32_t Ibus; // 总线电流
    }I_Def;
    
}Foc_DataDef;


extern Foc_DataDef Foc_Data;



uint16_t ADC_Read(ADC_HandleTypeDef AdcNum, uint32_t Channel);
int PT_TASK_Test(void);

int PT_TASK_FOC(void);

void FocInit(void);
float velocityOpenloop(float target_velocity);
// void FocLoop(void);
#endif
