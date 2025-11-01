/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-05-08 20:07:45
 * LastEditTime: 2025-05-29 21:52:44
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \g474app\My_Lib\st7789v\st7789v.c
 * @YJRQZ777
***************************************************************************************************/

#include "st7789v/st7789v.h"

#include "st7789v/font.h"

/***************************************************************************************************
 * 功能描述: SPI 发送字节函数
 * 输入参数: TxData    要发送的数据size    发送数据的字节大小
 * 输出参数: 
 * 返 回 值: 0:写入成功,其他:写入失败
 * 其它说明: 
 * param {uint8_t} TxData
 * param {uint16_t} size
***************************************************************************************************/
uint8_t SPI_WriteByte(uint8_t TxData, uint16_t size)
{
    return HAL_SPI_Transmit(&hspi3, &TxData, size, HAL_MAX_DELAY);
   	// return HAL_SPI_Transmit_DMA(&hspi3,&TxData,size);
}

/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint8_t} cmd
***************************************************************************************************/
static void LCD_Write_Cmd(uint8_t cmd)
{
    LCD_DC(CMD);
    SPI_WriteByte(cmd, 1);
}


/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint8_t} dat
***************************************************************************************************/
static void LCD_Write_Data(uint8_t dat)
{
    LCD_DC(DATA);
    SPI_WriteByte(dat, 1);
}
/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint16_t} dat
***************************************************************************************************/
void LCD_Write_Data2Bytes(uint16_t dat)
{
    LCD_DC(DATA);
	LCD_Write_Data(dat>>8);
	LCD_Write_Data(dat);
}
/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint16_t} x1
 * param {uint16_t} y1
 * param {uint16_t} x2
 * param {uint16_t} y2
***************************************************************************************************/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_Write_Cmd(0x2a);//列地址设置
		LCD_Write_Data2Bytes(x1+52);
		LCD_Write_Data2Bytes(x2+52);
		LCD_Write_Cmd(0x2b);//行地址设置
		LCD_Write_Data2Bytes(y1+40);
		LCD_Write_Data2Bytes(y2+40);
		LCD_Write_Cmd(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_Write_Cmd(0x2a);//列地址设置
		LCD_Write_Data2Bytes(x1+53);
		LCD_Write_Data2Bytes(x2+53);
		LCD_Write_Cmd(0x2b);//行地址设置
		LCD_Write_Data2Bytes(y1+40);
		LCD_Write_Data2Bytes(y2+40);
		LCD_Write_Cmd(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_Write_Cmd(0x2a);//列地址设置
		LCD_Write_Data2Bytes(x1+40);
		LCD_Write_Data2Bytes(x2+40);
		LCD_Write_Cmd(0x2b);//行地址设置
		LCD_Write_Data2Bytes(y1+53);
		LCD_Write_Data2Bytes(y2+53);
		LCD_Write_Cmd(0x2c);//储存器写
	}
	else
	{
		LCD_Write_Cmd(0x2a);//列地址设置
		LCD_Write_Data2Bytes(x1+40);
		LCD_Write_Data2Bytes(x2+40);
		LCD_Write_Cmd(0x2b);//行地址设置
		LCD_Write_Data2Bytes(y1+52);
		LCD_Write_Data2Bytes(y2+52);
		LCD_Write_Cmd(0x2c);//储存器写
	}
}
/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint8_t} Dir_Mode
***************************************************************************************************/
void ST7789V_SetDir(uint8_t Dir_Mode)
{
    LCD_Write_Cmd(0x36); /*显示方向*/
    if (Dir_Mode == 0)
        LCD_Write_Data(0x00);
    else if (Dir_Mode == 1)
        LCD_Write_Data(0xC0);
    else if (Dir_Mode == 2)
        LCD_Write_Data(0x70);
    else
        LCD_Write_Data(0xA0);
}
/***************************************************************************************************
 * 功能描述: 以一种颜色清空LCD屏
 * 输入参数: color —— 清屏颜色(16bit)
 * 输出参数: 
 * 返 回 值: none
 * 其它说明: 
 * param {uint16_t} color
// ***************************************************************************************************/
// void LCD_Clear(uint16_t color)
// {
//     LCD_Address_Set(0, 0, 240, 135);
//     for (uint32_t i = 0; i < LCD_BUFF_SIZE; i++)
//     {
				
//         uint8_tbuf[i * 2] = (color >> 8) & 0xFF;
//         uint8_tbuf[i * 2 + 1] = color & 0xFF;
			
//     }
//     // HAL_SPI_Transmit_DMA(&hspi3, buf,LCD_BUFF_SIZE*2);
// }

// void LCD_ConvertAndSendDMA()
// {
//     // 设置数据命令模式为 DATA
//     LCD_DC(DATA);
//     // 使用 DMA 发送 8 位数据缓冲区
//     HAL_SPI_Transmit_DMA(&hspi3, uint8_tbuf, LCD_W * LCD_H * 2);
//     // 等待 DMA 传输完成
//     // while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//     // {
//     //     /* code */
//     // }
    
// }





/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint16_t} x1
 * param {uint16_t} y1
 * param {uint16_t} x2
 * param {uint16_t} y2
 * param {uint16_t} color
***************************************************************************************************/
// void LCD_color_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
// {
//     uint16_t width = (x2 - x1) + 1;
//     uint16_t height = (y2 - y1) + 1;
//     uint16_t buf_size2 = (width * height);

//     LCD_Address_Set(x1, y1, x2, y2);
    
//     for (uint32_t i = 0; i < buf_size2; i++)
//     {
				
//         buf[i * 2] = (color >> 8) & 0xFF;
//         buf[i * 2 + 1] = color & 0xFF;
			
//     }
//     LCD_DC(DATA);
//     HAL_SPI_Transmit(&hspi3, buf,buf_size2*2,1000);
// }

//void LCD_color_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
//{
//    uint16_t width = x2 - x1;
//    uint16_t height = y2 - y1;
//    uint32_t buf_size = width * height * 2; // 16-bit color, 2 bytes per pixel
//    uint8_t *buf = (uint8_t *)malloc(buf_size);

//    LCD_DC(DATA);
//    LCD_Address_Set(x1, y1, x2, y2);
//    HAL_SPI_Transmit(&hspi3, buf, buf_size * 2, 1000);

//    free(buf);
//}

void LCD_color_point(uint16_t x1, uint16_t y1, uint16_t color)
{

    LCD_Address_Set(x1, y1, x1, y1);
    LCD_Write_Data2Bytes(color);
}


#define DUBEG 0

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_Write_Data2Bytes(color);
		}
	} 					  	    
}

/**
 * @brief   LCD初始化
 * @param   none
 * @return  none
 */

void st7789v_init(void)
{
//    LCD_BLK(0);
    LCD_CS(0);
    HAL_Delay(100);
    //
    //		HAL_Delay();
    LCD_RST(1);
    HAL_Delay(100);
    LCD_RST(0);
    HAL_Delay(100);
    LCD_RST(1);
    HAL_Delay(100);
    /* 初始化和LCD通信的引脚 */
    //    HAL_Delay(120);

    /* 关闭睡眠模式 */
    LCD_Write_Cmd(0x11);
    HAL_Delay(120);
    ST7789V_SetDir(USE_HORIZONTAL);
//    LCD_Write_Cmd(0x36);
//    LCD_Write_Data(0x00);



    LCD_Write_Cmd(0x3a);
    LCD_Write_Data(0x05); // 0x55
    //--------------------------------ST7789V Frame rate setting-----------------

    LCD_Write_Cmd(0xb2);
    LCD_Write_Data(0x0c);
    LCD_Write_Data(0x0c);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x33);
    LCD_Write_Data(0x33);
    LCD_Write_Cmd(0xb7);
    LCD_Write_Data(0x35);
    //---------------------------------ST7789V Power setting---------------------

    LCD_Write_Cmd(0xbb);
    LCD_Write_Data(0x19); // 0x28
    LCD_Write_Cmd(0xc0);
    LCD_Write_Data(0x2c);
    LCD_Write_Cmd(0xc2);
    LCD_Write_Data(0x01);
    LCD_Write_Cmd(0xc3);
    LCD_Write_Data(0x12); // 0x10
    LCD_Write_Cmd(0xc4);
    LCD_Write_Data(0x20);
    LCD_Write_Cmd(0xc6);
    LCD_Write_Data(0x0f);
    LCD_Write_Cmd(0xd0);
    LCD_Write_Data(0xa4);
    LCD_Write_Data(0xa1);
    //--------------------------------ST7789V gamma setting----------------------

    LCD_Write_Cmd(0xe0);
    LCD_Write_Data(0xd0);
    LCD_Write_Data(0x04); // 0x00
    LCD_Write_Data(0x0d); // 0x02
    LCD_Write_Data(0x11); // 0x07
    LCD_Write_Data(0x13); // 0x0a
    LCD_Write_Data(0x2b); // 0x28
    LCD_Write_Data(0x3f); // 0x32
    LCD_Write_Data(0x54); // 0x44
    LCD_Write_Data(0x4c); // 0x42
    LCD_Write_Data(0x18); // 0x06
    LCD_Write_Data(0x0d); // 0x0e
    LCD_Write_Data(0x0b); // 0x12
    LCD_Write_Data(0x1f); // 0x14
    LCD_Write_Data(0x23); // 0x17
    LCD_Write_Cmd(0xe1);
    LCD_Write_Data(0xd0); // 0xd0
    LCD_Write_Data(0x04); // 0x00
    LCD_Write_Data(0x0c); // 0x02
    LCD_Write_Data(0x11); // 0x07
    LCD_Write_Data(0x13); // 0x0a
    LCD_Write_Data(0x2c); // 0x28
    LCD_Write_Data(0x3f); // 0x31
    LCD_Write_Data(0x44); // 0x54
    LCD_Write_Data(0x51); // 0x47
    LCD_Write_Data(0x2f); // 0x0e
    LCD_Write_Data(0x1f); // 0x1c
    LCD_Write_Data(0x1f); // 0x17
    LCD_Write_Data(0x20); // 0x1b
    LCD_Write_Data(0x23); // 0x1e

    LCD_Write_Cmd(0x21);
    LCD_Write_Cmd(0x29);

    // LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    // HAL_Delay(1000);
    // LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    // HAL_Delay(1000);
    // // LCD_ShowPicture(0, 0, 240, 135, (uint8_t *)gImage_fu);

    // Draw_Circle(120, 67, 50, RED);
    LCD_ShowChineseTEST(0, 0, "FOC", WHITE, BLACK, 80,135, 0);
	HAL_Delay(10);
	LCD_ShowString(0, 0, "YJRQZ777", WHITE, BLACK, 16, 1);


	// HAL_Delay(1000);

    LCD_Fill(0, 0, 240, 135,WHITE);
    // HAL_Delay(100);
    // LCD_Clear(YELLOW);
    // HAL_Delay(100);

}



/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_Write_Data2Bytes(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}



/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_Write_Data2Bytes(fc);
						else LCD_Write_Data2Bytes(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_Write_Data2Bytes(fc);
						else LCD_Write_Data2Bytes(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_Write_Data2Bytes(fc);
						else LCD_Write_Data2Bytes(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			printf("32:%d,%d,%x,%X\n",HZnum,TypefaceNum,tfontTEST[k].Index[0],*(s));
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_Write_Data2Bytes(fc);
						else LCD_Write_Data2Bytes(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/***************************************************************************************************
 * 功能描述: 
 * 输入参数: 
 * 输出参数: 
 * 返 回 值: 
 * 其它说明: 
 * param {uint16_t} x
 * param {uint16_t} y
 * param {uint8_t} *s
 * param {uint16_t} fc
 * param {uint16_t} bc
 * param {uint8_t} sizey
 * param {uint8_t} mode
***************************************************************************************************/
void LCD_ShowChineseTEST(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizeW,uint8_t sizeH,uint8_t mode)
{
	uint16_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizeW*2/16)*sizeH;
	HZnum=sizeof(tfontTEST)/sizeof(typFNT_GBTEST);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		// printf("%d,%d,%x,%X\n",HZnum,TypefaceNum,tfontTEST[k].Index[0],*(s));
		if ((tfontTEST[k].Index[0]==*(s+k)))
		{ 	
			// printf("%d,%d,%x,%X\n",HZnum,TypefaceNum,tfontTEST[k].Index[0],*(s+k));
			LCD_Address_Set(x+sizeW*k,y,x+(80*(k+1))-1,y+(135)-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					// if(tfontTEST[k].Msk[i]&(0x01<<j))LCD_Write_Data2Bytes(fc);
					// else LCD_Write_Data2Bytes(bc);
					// m++;
					// if(m%sizeW==0)
					// {
					// 	m=0;
					// 	break;
					// }

					if(!mode)//非叠加方式
					{
						if(tfontTEST[k].Msk[i]&(0x01<<j))LCD_Write_Data2Bytes(fc);
						else LCD_Write_Data2Bytes(bc);
						m++;
						if(m%sizeW==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfontTEST[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizeW)
						{
							x=x0;
							y++;
							break;
						}
					}


				}
			}




		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}
/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}
/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_Write_Data2Bytes(fc);
				else LCD_Write_Data2Bytes(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_Write_Data(pic[k*2]);
			LCD_Write_Data(pic[k*2+1]);
			k++;
		}
	}			
}
