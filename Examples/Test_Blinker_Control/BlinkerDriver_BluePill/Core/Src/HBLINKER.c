#include "HBLINKER.h"
#include "HBLINKER_cfg.h"
#include "stm32f1xx_hal.h"

extern HBLINKER_strBLINKERCfg_t HBLINKER_strBLINKERCfg[HBLINKER_NUM_OF_BLINKERS];

// Global state for interrupt mode (volatile for interrupt safety)
static volatile HBLINKER_enuBlinkerState_t blinker_state[HBLINKER_NUM_OF_BLINKERS] = {HBLINKER_enuBlinker_OFF};
static volatile uint32_t last_interrupt_time[HBLINKER_NUM_OF_BLINKERS] = {0};
static volatile uint8_t pending_debounce[HBLINKER_NUM_OF_BLINKERS] = {0};
#define DEBOUNCE_MS 50

void HBLINKER_vInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    for (uint8_t iterator = 0; iterator < HBLINKER_NUM_OF_BLINKERS; iterator++) 
    {
        // Configure pins based on mode
        GPIO_InitStruct.Mode = (HBLINKER_strBLINKERCfg[iterator].HBLINKER_enuMode == HBLINKER_enuMode_INTERRUPT) 
                             ? GPIO_MODE_IT_FALLING : GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = (HBLINKER_strBLINKERCfg[iterator].HBLINKER_enuBLINKERConnection == HBLINKER_enuBLINKER_INTERNAL_PULL_UP) 
                             ? GPIO_PULLUP : GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

        // Configure first pin (PA2 for LEFT)
        GPIO_InitStruct.Pin = HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinL;
        HAL_GPIO_Init(HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PortL, &GPIO_InitStruct);
        
        // Configure second pin (PA1 for RIGHT)
        if (HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinR != 0)
        {
            GPIO_InitStruct.Pin = HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinR;
            HAL_GPIO_Init(HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PortR, &GPIO_InitStruct);
        }

        // Enable EXTI interrupts if in interrupt mode
        if (HBLINKER_strBLINKERCfg[iterator].HBLINKER_enuMode == HBLINKER_enuMode_INTERRUPT)
        {
            if (HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinR == GPIO_PIN_1)
            {
                HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(EXTI1_IRQn);
            }
            if (HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinL == GPIO_PIN_2)
            {
                HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(EXTI2_IRQn);
            }
        }
    }
}

static HBLINKER_enuErrorStatus_t HBLINKER_enuGetBLINKERStateWithDebounce(GPIO_TypeDef *Copy_GPIO_PortL, uint16_t Copy_GPIO_PinL, 
                                                                         GPIO_TypeDef *Copy_GPIO_PortR, uint16_t Copy_GPIO_PinR, 
                                                                         uint8_t *Add_u8PinValue)
{
    HBLINKER_enuErrorStatus_t Ret_enuErrorStatus = HBLINKER_enuError_OK;
    
    if (Add_u8PinValue == NULL)
    {
        return HBLINKER_enuError_NULL_POINTER;
    }
    
    // Read both pins
    uint8_t pinL = HAL_GPIO_ReadPin(Copy_GPIO_PortL, Copy_GPIO_PinL);
    uint8_t pinR = HAL_GPIO_ReadPin(Copy_GPIO_PortR, Copy_GPIO_PinR);
    HAL_Delay(20);
    uint8_t pinL_second = HAL_GPIO_ReadPin(Copy_GPIO_PortL, Copy_GPIO_PinL);
    uint8_t pinR_second = HAL_GPIO_ReadPin(Copy_GPIO_PortR, Copy_GPIO_PinR);
    
    if (pinL != pinL_second || pinR != pinR_second)
    {
        Ret_enuErrorStatus = HBLINKER_enuError_NOK;
    }
    else
    {
        if (pinL == GPIO_PIN_RESET && pinR == GPIO_PIN_SET)
            *Add_u8PinValue = HBLINKER_enuBlinker_RIGHT; // ON Right
        else if (pinL == GPIO_PIN_SET && pinR == GPIO_PIN_RESET)
            *Add_u8PinValue = HBLINKER_enuBlinker_LEFT; // ON Left
        else
            *Add_u8PinValue = HBLINKER_enuBlinker_OFF; // OFF
    }
    
    return Ret_enuErrorStatus;
}

// Interrupt handlers for EXTI1 (PA1) and EXTI2 (PA2)
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (uint8_t iterator = 0; iterator < HBLINKER_NUM_OF_BLINKERS; iterator++)
    {
        if ((GPIO_Pin == HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinR) || 
            (GPIO_Pin == HBLINKER_strBLINKERCfg[iterator].HBLINKER_GPIO_PinL))
        {
            last_interrupt_time[iterator] = HAL_GetTick();
            pending_debounce[iterator] = 1; // Flag for debounce check
        }
    }
}

HBLINKER_enuErrorStatus_t HBLINKER_enuGetBLINKERState(uint8_t Copy_u8BLINKERName, uint8_t *Add_u8State)
{
    HBLINKER_enuErrorStatus_t Ret_enuErrorStatus = HBLINKER_enuError_NOK;
    
    if (Copy_u8BLINKERName >= HBLINKER_NUM_OF_BLINKERS)
    {
        return HBLINKER_enuError_INVALID_NUM_OF_BLINKERS;
    }
    
    if (Add_u8State == NULL)
    {
        return HBLINKER_enuError_NULL_POINTER;
    }
    
    if (HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_enuMode == HBLINKER_enuMode_INTERRUPT)
    {
        if (pending_debounce[Copy_u8BLINKERName] && 
            (HAL_GetTick() - last_interrupt_time[Copy_u8BLINKERName] >= DEBOUNCE_MS))
        {
            uint8_t pinL = HAL_GPIO_ReadPin(HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PortL,
                                            HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PinL);
            uint8_t pinR = HAL_GPIO_ReadPin(HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PortR,
                                            HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PinR);

            if (HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_enuBLINKERDebounce == HBLINKER_enuBLINKERDebounce_ON)
            {
                HAL_Delay(20); // Secondary check for stability
                uint8_t pinL_second = HAL_GPIO_ReadPin(HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PortL,
                                                       HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PinL);
                uint8_t pinR_second = HAL_GPIO_ReadPin(HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PortR,
                                                       HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PinR);
                if (pinL != pinL_second || pinR != pinR_second)
                {
                    Ret_enuErrorStatus = HBLINKER_enuError_NOK;
                    return Ret_enuErrorStatus;
                }
            }

            if (pinL == GPIO_PIN_RESET && pinR == GPIO_PIN_SET)
            {
                blinker_state[Copy_u8BLINKERName] = HBLINKER_enuBlinker_RIGHT;
            }
            else if (pinL == GPIO_PIN_SET && pinR == GPIO_PIN_RESET)
            {
                blinker_state[Copy_u8BLINKERName] = HBLINKER_enuBlinker_LEFT;
            }
            else
            {
                blinker_state[Copy_u8BLINKERName] = HBLINKER_enuBlinker_OFF;
            }
            pending_debounce[Copy_u8BLINKERName] = 0; // Clear debounce flag
        }
        *Add_u8State = blinker_state[Copy_u8BLINKERName];
        Ret_enuErrorStatus = HBLINKER_enuError_OK;
    }
    else
    {
        GPIO_TypeDef *Loc_GPIO_PortL = HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PortL;
        uint16_t Loc_GPIO_PinL = HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PinL;
        GPIO_TypeDef *Loc_GPIO_PortR = HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PortR;
        uint16_t Loc_GPIO_PinR = HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_GPIO_PinR;
        HBLINKER_enuBLINKERDebounce_t Loc_BLINKERDebounce = HBLINKER_strBLINKERCfg[Copy_u8BLINKERName].HBLINKER_enuBLINKERDebounce;
        
        if (Loc_BLINKERDebounce == HBLINKER_enuBLINKERDebounce_ON)
        {
            Ret_enuErrorStatus = HBLINKER_enuGetBLINKERStateWithDebounce(Loc_GPIO_PortL, Loc_GPIO_PinL, 
                                                                         Loc_GPIO_PortR, Loc_GPIO_PinR, 
                                                                         Add_u8State);
        }
        else
        {
            uint8_t pinL = HAL_GPIO_ReadPin(Loc_GPIO_PortL, Loc_GPIO_PinL);
            uint8_t pinR = HAL_GPIO_ReadPin(Loc_GPIO_PortR, Loc_GPIO_PinR);
            if (pinL == GPIO_PIN_RESET && pinR == GPIO_PIN_SET)
                *Add_u8State = HBLINKER_enuBlinker_RIGHT; // ON Right
            else if (pinL == GPIO_PIN_SET && pinR == GPIO_PIN_RESET)
                *Add_u8State = HBLINKER_enuBlinker_LEFT; // ON Left
            else
                *Add_u8State = HBLINKER_enuBlinker_OFF; // OFF
            Ret_enuErrorStatus = HBLINKER_enuError_OK;
        }
    }
    
    return Ret_enuErrorStatus;
}