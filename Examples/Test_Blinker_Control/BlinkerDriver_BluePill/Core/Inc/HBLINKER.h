#ifndef BLINKER_H
#define BLINKER_H

#include "stdint.h"
#include "stm32f1xx_hal.h"

typedef enum {
    HBLINKER_enuBlinker_OFF,
    HBLINKER_enuBlinker_RIGHT,
    HBLINKER_enuBlinker_LEFT
} HBLINKER_enuBlinkerState_t;

typedef enum {
    HBLINKER_enuError_OK,
    HBLINKER_enuError_NOK,
    HBLINKER_enuError_INVALID_NUM_OF_BLINKERS,
    HBLINKER_enuError_INVALID_BLINKER_CONNECTION,
    HBLINKER_enuError_NULL_POINTER
} HBLINKER_enuErrorStatus_t;

typedef enum {
    HBLINKER_enuBLINKER_EXTERNAL_PULL_DOWN,
    HBLINKER_enuBLINKER_EXTERNAL_PULL_UP,
    HBLINKER_enuBLINKER_INTERNAL_PULL_UP
} HBLINKER_enuBLINKERConnection_t;

typedef enum {
    HBLINKER_enuBLINKERDebounce_OFF,
    HBLINKER_enuBLINKERDebounce_ON
} HBLINKER_enuBLINKERDebounce_t;

typedef enum {
    HBLINKER_enuMode_POLLING,
    HBLINKER_enuMode_INTERRUPT
} HBLINKER_enuMode_t;

void HBLINKER_vInit(void);
HBLINKER_enuErrorStatus_t HBLINKER_enuGetBLINKERState(uint8_t Copy_u8BLINKERName, uint8_t *Add_u8State);

#endif // BLINKER_H