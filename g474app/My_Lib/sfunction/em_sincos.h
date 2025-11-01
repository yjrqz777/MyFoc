
/**
  ******************************************************************************
  * @file    mc_math.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MC_Math
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef EM_SINCOS_H_
#define EM_SINCOS_H_

/* Includes ------------------------------------------------------------------*/
#include "data_type.h"
/**
  * @brief  Trigonometrical functions type definition
  */
int16 EmSin( int16 hAngle );
int16 EmCos( int16 hAngle );


#endif /* MC_MATH_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
