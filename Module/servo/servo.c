/**
 * @note 此文件根据市面上的通用舵机编写
 * @note 要求的PWM波的频率为50Hz,即20ms,此处应在CubeMX内通过配置时钟频率和PSC值完成
 * @note 占空比为0.5ms~2.5ms,对应-90°到90°
 */

#include "servo.h"

/**
 * @brief 注册舵机实例
 */
Servo_Instance *Servo_Register(Servo_Config_s* config)
{
    /* 参数检验 */
    if(config == NULL)
        return NULL;
    else if(config->tim == NULL)
        return NULL;

    /* 分配空间 */
    Servo_Instance *instance = (Servo_Instance *)pvPortMalloc(sizeof(Servo_Instance));
    if(instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(Servo_Instance));

    /* 参数传递 */
    instance->tim = config->tim;
    instance->channel = config->channel;
    instance->arr = config->arr;
    instance->ccr = 0;

    /* 返回句柄 */
    return instance;
}

/**
 * @brief 舵机实例初始化
 */
void Servo_Init(Servo_Instance *instance)
{ 
    HAL_TIM_PWM_Start(instance->tim, instance->channel);
}

/**
 * @brief 设置舵机角度(角度值,单位为°)
 */
void Servo_SetAngle(Servo_Instance *instance, float angle)
{
    instance->ccr = (instance->arr + 1) * (angle / 1800.0f + 0.075); 
    __HAL_TIM_SET_COMPARE(instance->tim, instance->channel, instance->ccr);
}
