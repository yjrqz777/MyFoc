#include "ft6336.h"
#include "stdio.h"
// #include "i2c.h"

#include "i2c.h"

uint8_t do_i2cdetect_cmd()
{
    uint8_t address;

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16)
    {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++)
        {
            fflush(stdout);
            address = i + j;
            uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c2, (address << 1) | 1, 2, 500);
            if (ret == HAL_OK)
            {
                printf("%02x ", address);
            }
            else if (ret == HAL_TIMEOUT)
            {
                printf("UU ");
            }
            else
            {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
   return 0;
    // 		uint8_t data[8]={0};
    // 		HAL_I2C_Mem_Read(&hi2c2,(0x38 << 1) | 1,0x85,1,data, 1,1000);
	// 			printf("-%02X-\r\n",data[0]);
	// 		uint8_t data2 = 0xc8;
    // // uint8_t data23[2];
	// 		HAL_I2C_Mem_Write(&hi2c2, (0x38 << 1) | 0, 0x85, 1, &data2, 1, 1000);
    // //		for(int i = 0;i<1;i++)
	// 			HAL_I2C_Mem_Read(&hi2c2,(0x38 << 1) | 1,0x85,1,data, 1,1000);
    // 		printf("-%02X-\r\n",data[0]);
}

void ft6336_set_sleep_time(uint8_t time)
{
    uint8_t data2 = time;
    // uint8_t data23[2];
    HAL_I2C_Mem_Write(&hi2c2, (0x38 << 1) | 0, 0x87, 1, &data2, 1, 1000);
    // HAL_I2C_Mem_Read(&hi2c2,(0x38 << 1) | 1,0x87,1,data23, 1,1000);
    // printf("-%02X-\r\n",data23[0]);
}

/*
0x03	1 Byte	只读(RO)	P1_XH	第1点的X坐标高4位	bit7-bit6为第1点触摸事件标志，bit3-bit0为第1点的X坐标高4位。
0x04	1 Byte	只读(RO)	P1_XL	第1点的X坐标低8位
0x05	1 Byte	只读(RO)	P1_YH	第1点的Y坐标高4位	bit7-bit4为第1点的ID，bit3-bit0为第1点的Y坐标高4位。
0x06	1 Byte	只读(RO)	P1_YL	第1点的Y坐标低8位

*/
uint8_t ft6336_enable()
{
    uint8_t data[1] = {0};
    HAL_I2C_Mem_Read(&hi2c2, (FT6636ADDR << 1) | 1, 0x02, 1, data, 1, 1000);
    if (data[0] > 0)
        return 1;
    else
        return 0;
}

void ft6336_read_xy(uint16_t *x, uint16_t *y)
{
    uint8_t data[4] = {0};

    HAL_I2C_Mem_Read_DMA(&hi2c2, (FT6636ADDR << 1) | 1, 0x03, 1, data, 4);

    while (HAL_I2C_IsDeviceReady(&hi2c2, (FT6636ADDR << 1) | 1, 2, 50))
        ;
    ////		printf("-%02x-%02x-%02x-%02x", data[0], data[1],data[2],data[3]);
    //		for(int i=0;i<100;i++)
    //			for(int j=0;j<100;j++)
    //				for(int j=0;j<10;j++);

    //		HAL_I2C_Mem_Read(&hi2c2, (FT6636ADDR << 1) | 1, 0x03, 1, data, 4,1000);
    *x = ((data[2] & 0x0f) << 8) + data[3];
    *y = 240 - ((data[0] & 0x0f) << 8 | data[1]);
//		*y = (((data[0] & 0x0f) << 8) + data[1]);
    printf("x: %d, y: %d\r\n", *x, *y);

    //    printf("\r\n");
    //		return data;
}
uint16_t ft6336_read_x()
{
    uint8_t data[2] = {0};
    uint8_t data2[2] = {0};

    uint8_t ret_x = HAL_I2C_Mem_Read_DMA(&hi2c2, (FT6636ADDR << 1) | 1, 0x05, 1, data, 2);
    //    HAL_I2C_Mem_Read(&hi2c2, (FT6636ADDR << 1) | 1, 0x06, 1, data2, 1,1000);
    //		printf("x:-%d-\r\n",ret_x);
    //		HAL_Delay(1);
    for (int i = 0; i < 100; i++)
        for (int j = 0; j < 100; j++)
            ;
    uint16_t x = (data[0] & 0x0f) << 8 | data[1];
    printf("x: %d ", x);
    return x;
}
uint16_t ft6336_read_y()
{
    uint8_t data3[2] = {0};
    uint8_t data4[2] = {0};
    uint8_t ret_x, ret_y;
    ret_x = HAL_I2C_Mem_Read_DMA(&hi2c2, (FT6636ADDR << 1) | 1, 0x03, 1, data3, 2);
    //    ret_y = HAL_I2C_Mem_Read_DMA(&hi2c2, (FT6636ADDR << 1) | 1, 0x04, 1, data4, 1);
    printf("-%d-%d-\r\n", ret_x, ret_y);
    uint16_t y = (data3[0] & 0x0f) << 8 | data3[1];
    printf("\r\n");
    for (int i = 7; i >= 0; i--)
    {
        printf("%d", (data3[0] >> i) & 0x01);
    }
    printf(" ");
    for (int i = 7; i >= 0; i--)
    {
        printf("%d", (data3[1] >> i) & 0x01);
    }
    printf(" ");
    y = 240 - y;
    printf("y: %d,data3[0]:%02x,data4[0]:%02x", y, data3[0], data4[0]);
    printf("\r\n");
    return y;
}