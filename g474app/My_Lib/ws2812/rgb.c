#include "rgb.h"
#include "main.h"
#include "tim.h"


uint32_t send_Buf[NUM] = {0};


uint32_t rgb2grb(uint32_t rgb)
{
    uint32_t r = (rgb >> 16) & 0xFF;   // 提取红色分量  
    uint32_t g = (rgb >> 8) & 0xFF;     // 提取绿色分量  
    uint32_t b = rgb & 0xFF;             // 提取蓝色分量  
    uint32_t grb = (g << 16) | (r << 8) | b;  
//	printf("\n%d,\n%d,\n%d,\n%d\n",r,g,b,grb);
	return grb;
}


void WS_Load(void)
{
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)send_Buf, NUM);
}


void WS_CloseAll(void)
{
	uint16_t i;
	for (i = 0; i < PIXEL_NUM * 24; i++)
		send_Buf[i] = WSL; // 写入逻辑0的占空比
	for (i = PIXEL_NUM * 24; i < NUM; i++)
		send_Buf[i] = 0; // 占空比比为0，全为低电平
	WS_Load();
}


void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor)
{
	uint8_t i;
	if (n < PIXEL_NUM)
	{
		for (i = 0; i < 24; ++i)
			send_Buf[24 * n + i] = (((GRBColor << i) & 0X800000) ? WSH : WSL);
	}
//	for (int i = 0; i < NUM; i++)
//	printf("send_Buf[%d]=%d\n",i,send_Buf[i]);
}
 

void WS281x_SetAll(uint32_t GRBColor)
{
	uint8_t i;
	for(int j=0; j<PIXEL_NUM;j++)
		for (i = 0; i < 24 ; i++)
			send_Buf[j*24 + i] = (((GRBColor << i) & 0X800000) ? WSH : WSL);
//	for (int i = 0; i < NUM; i++)
//	printf("send_Buf[%d]=%d\n",i,send_Buf[i]);
}

uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue)
{
	return green << 16 | red << 8 | blue;
}

uint32_t Wheel(uint8_t WheelPos)
{
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85)
	{
		return WS281x_Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if (WheelPos < 170)
	{
		WheelPos -= 85;
		return WS281x_Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return WS281x_Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}



void rainbowCycle(uint8_t wait)
{
	uint32_t timestamp = HAL_GetTick();
	uint16_t i;
	static uint8_t j;
	static uint32_t next_time = 0;

	static uint8_t loop = 0;
	if (loop == 0)
		next_time = timestamp;
	loop = 1; //首次调用初始化

	if ((timestamp > next_time)) // && (timestamp - next_time < wait*5))
	{
		j++;
		next_time = timestamp + wait;
		for (i = 0; i < PIXEL_NUM; i++)
		{
			WS281x_SetPixelColor(i, Wheel(((i * 256 / (PIXEL_NUM)) + j) & 255));
		}
	}
	WS_Load();
}

 //  DMA 传输完成回调函数
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop_DMA(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_1);
}
 
 
 