#ifndef AC_DRIVER_H
#define AC_DRIVER_H

#include "stm32f1xx_hal.h"

// Direction options
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1
} MotorDirection;

// Motor A: connected to TIM3_CH1 (PA6), IN1 (PA0), IN2 (PA1)
// Motor B: connected to TIM3_CH2 (PA7), IN3 (PA2), IN4 (PA3)

void Motor_Init(void);

void MotorA_SetDirection(MotorDirection dir);
void MotorA_SetSpeed(uint8_t percent); // 0–100

void MotorB_SetDirection(MotorDirection dir);
void MotorB_SetSpeed(uint8_t percent); // 0–100

#endif // MOTOR_H
