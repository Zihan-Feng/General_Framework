#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 舵机实例结构体 */
typedef struct servo_struct
{
    TIM_HandleTypeDef *tim;
    uint32_t channel;
    uint32_t arr;
    uint32_t ccr;
    float duty;
} Servo_Instance;

/* 舵机配置结构体 */
typedef struct servo_config_struct
{
    TIM_HandleTypeDef *tim;
    uint32_t channel;
    uint32_t arr;
} Servo_Config_s;

Servo_Instance *Servo_Register(Servo_Config_s* config);
void Servo_Init(Servo_Instance *instance);
void Servo_SetAngle(Servo_Instance *instance, float angle);

#ifdef __cplusplus
}
#endif

#endif