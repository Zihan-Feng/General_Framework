/**
 * @file encoder.h
 * @author ZZT (2863861004@qq.com),STHY
 * @brief 
 * @version 0.2
 * @date 2025-6-25
 * 
 * @copyright Copyright (c) 2025
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#ifndef ENCODER_H 
#define ENCODER_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "rtos_interface.h"

#include "bsp_usart.h"
#include "bsp_log.h"

#include "topics.h"
#include "data_type.h"
#include "crc_util.h"
/*-----------------------------------macro------------------------------------*/
extern int32_t Encoder_count_;
/*----------------------------------typedef-----------------------------------*/
typedef struct
{
    Uart_Instance_t *Encoder_uart_instance;
    rtos_for_module_t *rtos_for_Encoder;
    Publisher *Encoder_pub;
    pub_Encoder_Data pub_data;
    uint8_t (*Encoder_task)(void* Encoder_instance);
    uint8_t (*get_data)(uint8_t *data,pub_Encoder_Data *Encoder_Data);
    uint8_t (*Encoder_deinit)(void* Encoder_instance);
}Encoder_Instance_t;


/*----------------------------------function----------------------------------*/

/**
 * @brief 
 * 
 * @param Encoder_uart_instance 
 * @param queue_length 
 * @return Encoder_Instance_t* 
 */
Encoder_Instance_t* Encoder_init(Uart_Instance_t *Encoder_uart_instance,uint32_t queue_length);


/**
 * @brief 
 * 
 * @param Encoder_instance 
 * @return uint8_t 
 */
uint8_t Encoder_Task(void* Encoder_instance);

uint8_t Encoder_Send_Write_0x0007(UART_HandleTypeDef *uart_handle, uint8_t addr, uint16_t value);

uint8_t Encoder_Parse_Write_0x0007_Response(uint8_t *data, uint16_t len);


/**
 * @brief 
 * 
 * @param uart_instance 
 * @param data_len 
 * @return uint8_t 
 */
uint8_t Encoder_Uart_Rx_Callback(void *uart_instance,uint16_t data_len);


/**
 * @brief 
 * 
 * @param Encoder_instance 
 * @return uint8_t 
 */
uint8_t Encoder_DeInit(void *Encoder_instance);

#ifdef __cplusplus
}
#endif

#endif	/* Encoder_H */
