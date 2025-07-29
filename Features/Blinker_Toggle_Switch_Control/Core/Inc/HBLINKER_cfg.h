#ifndef BLINKER_CFG_H
#define BLINKER_CFG_H

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "HBLINKER.h"

#define HBLINKER_NUM_OF_BLINKERS 1

#define HBLINKER_BLINKER_TOGGLE 0
// #define HBLINKER_BLINKER_STOP 1
// #define HBLINKER_BLINKER_ALERT 2

typedef struct {
    GPIO_TypeDef *HBLINKER_GPIO_PortL;    // First pin port (LEFT)
    uint16_t HBLINKER_GPIO_PinL;          // First pin (PA2)
    GPIO_TypeDef *HBLINKER_GPIO_PortR;    // Second pin port (RIGHT)
    uint16_t HBLINKER_GPIO_PinR;         // Second pin (PA1)
    HBLINKER_enuBLINKERConnection_t HBLINKER_enuBLINKERConnection;
    HBLINKER_enuBLINKERDebounce_t HBLINKER_enuBLINKERDebounce;
    HBLINKER_enuMode_t HBLINKER_enuMode; // Polling or Interrupt
} HBLINKER_strBLINKERCfg_t;

#endif // BLINKER_CFG_H