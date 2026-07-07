#include "bsp_hall.h"

uint8_t BspHall_ReadA(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
}

uint8_t BspHall_ReadB(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
}

uint8_t BspHall_ReadC(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);
}

BspHallPins_t BspHall_ReadPins(void)
{
    BspHallPins_t pins;

    pins.a = BspHall_ReadA();
    pins.b = BspHall_ReadB();
    pins.c = BspHall_ReadC();

    return pins;
}

uint8_t BspHall_GetState(void)
{
    BspHallPins_t pins = BspHall_ReadPins();
    return (uint8_t)(pins.a | (pins.b << 1) | (pins.c << 2));
}

uint8_t BspHall_IsValidState(uint8_t state)
{
    state &= 0x07u;
    return (state != 0x00u) && (state != 0x07u);
}
