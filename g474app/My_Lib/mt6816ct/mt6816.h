/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-08-10 19:39:03
 * LastEditTime: 2025-08-10 20:05:41
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \g474app\My_Lib\mt6816ct\mt6816.h
 * @YJRQZ777
***************************************************************************************************/
#ifndef MT_6816_H
#define MT_6816_H
#include "main.h"


#include <stdbool.h>	
#include <string.h>		
#include <stdlib.h>		
#include <stdio.h>		
#include <spi.h>		
#include <main.h>		

#define MT6816_SPI_CS_H()	     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1) 
#define MT6816_SPI_CS_L()		 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0) 
#define MT6816_SPI_Get_HSPI		    (hspi1)
#define MT6816_Mode_SPI		        (0x03)	


typedef struct{
	uint16_t	sample_data;	
	uint16_t	angle;				
	bool		no_mag_flag;	
	bool		pc_flag;			
}MT6816_SPI_Signal_Typedef;

void REIN_MT6816_SPI_Signal_Init(void);		
void RINE_MT6816_SPI_Get_AngleData(void);	

typedef struct{
	uint16_t	angle_data;			
	uint16_t	rectify_angle;		
	bool			rectify_valid;		
}MT6816_Typedef;

extern MT6816_Typedef	mt6816;
					
float REIN_MT6816_Get_AngleData();

extern int PT_TASK_mt6816();

#endif
