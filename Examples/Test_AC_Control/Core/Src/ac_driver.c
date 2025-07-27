#include "ac_driver.h"
#include "stm32f1xx_hal.h"
// Assumes TIM3 is initialized in CubeMX
extern TIM_HandleTypeDef htim3;

void Motor_Init(void)
{
    // Start PWM timers
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Motor A
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Motor B
}

// ---- MOTOR A ----
void MotorA_SetDirection(MotorDirection dir)
{
    if (dir == MOTOR_DIR_FORWARD) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1 = HIGH
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2 = LOW
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // IN1 = LOW
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN2 = HIGH
    }
}

void MotorA_SetSpeed(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint32_t pulse = (uint32_t)(percent * 10); // if ARR = 999 (max 1000)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

// ---- MOTOR B ----
void MotorB_SetDirection(MotorDirection dir)
{
    if (dir == MOTOR_DIR_FORWARD) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // IN3 = HIGH
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // IN4 = LOW
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // IN3 = LOW
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // IN4 = HIGH
    }
}

void MotorB_SetSpeed(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint32_t pulse = (uint32_t)(percent * 10); // if ARR = 999
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}
