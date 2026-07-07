#ifndef __BSP_HALL_H__
#define __BSP_HALL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    uint8_t a;
    uint8_t b;
    uint8_t c;
} BspHallPins_t;

BspHallPins_t BspHall_ReadPins(void);
uint8_t BspHall_ReadA(void);
uint8_t BspHall_ReadB(void);
uint8_t BspHall_ReadC(void);
uint8_t BspHall_GetState(void);
uint8_t BspHall_IsValidState(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif
