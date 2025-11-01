/**
  ******************************************************************************
  * @file    mc_math.c
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
  */
/* Includes ------------------------------------------------------------------*/
#include "data_type.h"
#include "em_sincos.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

/* Private macro -------------------------------------------------------------*/
/*
SIN_COS_TABLE解析：
1. 表中共有256个元素，分别对应0~90°内256个角度的sin值
2. 表中元素的数据类型为定点数q1.15
   - 即16bit整数来表示浮点数，其中最高位bit15为符号位，bit14~0为小数位
   - 如，表中下标为7的元素(第一行最后一个)：0x057F(10进制：1407)
   - 对应角度为90°/256*7=2.461°，sin(2.461°)=0.0429；对应的q1.15为0x057F(10进制：1407)，1407/32768=0.0429
*/
#define SIN_COS_TABLE {\
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
    0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
    0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
    0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
    0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
    0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
    0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
    0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
    0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
    0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
    0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
    0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
    0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
    0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
    0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
    0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
    0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
    0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
    0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
    0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
    0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
    0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
    0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
    0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
    0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
    0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
    0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
    0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
    0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
    0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE}


#define SIN_MASK        0x0300u
#define U0_90           0x0200u
#define U90_180         0x0300u
#define U180_270        0x0000u
#define U270_360        0x0100u

/* Private variables ---------------------------------------------------------*/
const int16 hSin_Cos_Table[256] = SIN_COS_TABLE;


/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Sin(angle) and Cos(angle) in Trig_Components format
  */

/**********************************************************
int16 EmSin( int16 hAngle )
input: 
- 角度；单位：s16Degree；取值范围[-32768，32767], 对应[-pi, pi]或[-180°, 180°]
- 如：0 s16对应0，16384 s16对应pi/2，32767 s16对应pi，-32768对应-pi，-16384对应-pi/2
return: sin值，单位：q1.15
**********************************************************/
int16 EmSin( int16 hAngle )
{
  int32 shindex;
  uint16 uhindex;

  int16 hSin;

  /* hAngle：[-32768, 32767], 对应[-pi, pi]或[-180°, 180°] */
  /* shindex = 32768 + hAngle：[0, 65535] */
  /* [0, 16383]对应[180°, 270°] */
  /* [16384, 32767]对应[270°, 360°] */
  /* [32768, 49151]对应[0°, 90°] */
  /* [49152, 65535]对应[90°, 180°] */
  shindex = ( ( int32 )32768 + ( int32 )hAngle );
  uhindex = ( uint16 )shindex;
  
  /* uhindex /= 64: [0, 1023] */
  /* [0,    255]或[0x000, 0x0FF]对应[180°, 270°] */
  /* [256,  511]或[0x100, 0x1FF]对应[270°, 360°] */
  /* [512,  767]或[0x200, 0x2FF]对应[0°, 90°] */
  /* [768, 1023]或[0x300, 0x3FF]对应[90°, 180°] */
  uhindex /= ( uint16 )64;

  switch ( ( uint16 )( uhindex ) & SIN_MASK )
  {
    case U0_90:
		/* [512,  767]或[0x200, 0x2FF]对应[0°, 90°] */
		hSin = hSin_Cos_Table[( uint8 )( uhindex )];
		break;

    case U90_180:
		/* [768, 1023]或[0x300, 0x3FF]对应[90°, 180°] */
		/* 若a < 90°：sin(a + 90°) = sin(90° - a) */
		hSin = hSin_Cos_Table[( uint8 )( 0xFFu - ( uint8 )( uhindex ) )];	
		break;

    case U180_270:
		/* [0,	  255]或[0x000, 0x0FF]对应[180°, 270°] */
		/* 若a < 90°：sin(a + 180°) = -sin(a) */
		hSin = -hSin_Cos_Table[( uint8 )( uhindex )];
		break;

    case U270_360:
		/* [256,  511]或[0x100, 0x1FF]对应[270°, 360°] */
		/* 若a < 90°：sin(a + 270) = -sin(90° - a) */
		hSin =  -hSin_Cos_Table[( uint8 )( 0xFFu - ( uint8 )( uhindex ) )];
		break;
    default:
		break;
  }
  return ( hSin );
}

int16 EmCos( int16 hAngle )
{

  int32 shindex;
  uint16 uhindex;

  int16 hCos;

  /* 10 bit index computation  */
  shindex = ( ( int32 )32768 + ( int32 )hAngle );
  uhindex = ( uint16 )shindex;
  uhindex /= ( uint16 )64;

  switch ( ( uint16 )( uhindex ) & SIN_MASK )
  {
    case U0_90:
      hCos = hSin_Cos_Table[( uint8 )( 0xFFu - ( uint8 )( uhindex ) )];
      break;

    case U90_180:
      hCos = -hSin_Cos_Table[( uint8 )( uhindex )];
      break;

    case U180_270:
      hCos = -hSin_Cos_Table[( uint8 )( 0xFFu - ( uint8 )( uhindex ) )];
      break;

    case U270_360:
      hCos =  hSin_Cos_Table[( uint8 )( uhindex )];
      break;
    default:
      break;
  }
  return ( hCos );
}


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
