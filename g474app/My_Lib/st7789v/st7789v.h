/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-05-08 20:07:45
 * LastEditTime: 2025-05-11 18:36:18
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description: 
 * FilePath: \g474app\My_Lib\st7789v\st7789v.h
 * @YJRQZ777
***************************************************************************************************/
#ifndef __ST7789V_H
#define __ST7789V_H
#include "main.h"

#include "stdio.h"
#include "spi.h"
#include "gpio.h"





#define LCD_RST(x) HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin,(x?GPIO_PIN_SET:GPIO_PIN_RESET))
#define LCD_DC(x) HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,(x?GPIO_PIN_SET:GPIO_PIN_RESET))									
#define LCD_CS(x) HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,(x?GPIO_PIN_SET:GPIO_PIN_RESET))


#define CMD 0
#define DATA 1


#define WIDTH_OFFSET 		 0
#define HIGH_OFFSET 		 20

#define USE_HORIZONTAL 	 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 					 135
#define LCD_H 					 240
#else
#define LCD_W 					 240
#define LCD_H 					 135
#endif

#define LCD_BUFF_SIZE  (LCD_W*LCD_H) //240*135*2=64800


//画笔颜色
#define WHITE         	 (uint16_t)0xFFFF
#define BLACK         	 (uint16_t)0x0000	  
#define BLUE           	 (uint16_t)0x001F  
#define BRED             (uint16_t)0XF81F
#define GRED 			       (uint16_t)0XFFE0
#define GBLUE			       (uint16_t)0X07FF
#define RED           	 (uint16_t)0xF800
#define MAGENTA       	 (uint16_t)0xF81F
#define GREEN         	 (uint16_t)0x07E0
#define CYAN          	 (uint16_t)0x7FFF
#define YELLOW        	 (uint16_t)0xFFE0
#define BROWN 			     (uint16_t)0XBC40 //棕色
#define BRRED 			     (uint16_t)0XFC07 //棕红色
#define GRAY  			     (uint16_t)0X8430 //灰色
#define DARKBLUE      	 (uint16_t)0X01CF	//深蓝色
#define LIGHTBLUE      	 (uint16_t)0X7D7C	//浅蓝色  
#define GRAYBLUE       	 (uint16_t)0X5458 //灰蓝色
#define LIGHTGREEN     	 (uint16_t)0X841F //浅绿色
#define LGRAY 			     (uint16_t)0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        (uint16_t)0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           (uint16_t)0X2B12 //浅棕蓝色(选择条目的反色)



extern void st7789v_init(void);
// extern void LCD_color_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
extern void LCD_color_point(uint16_t x1, uint16_t y1,uint16_t color);

// extern void LCD_ConvertAndSendDMA();


void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//指定区域填充颜色
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);//在指定位置画一个点
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color);//在指定位置画一条线
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);//在指定位置画一个矩形
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color);//在指定位置画一个圆

void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示汉字串
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个12x12汉字
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个16x16汉字
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个24x24汉字
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示单个32x32汉字

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示一个字符
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示字符串
uint32_t mypow(uint8_t m,uint8_t n);//求幂
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//显示整数变量
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//显示两位小数变量

void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]);//显示图片
void LCD_ShowChineseTEST(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizeW,uint8_t sizeH,uint8_t mode);


//extern void LCD_color_fill_lvgl(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, lv_color_t * color);
#endif
