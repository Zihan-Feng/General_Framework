/**
 * @file Encoder.c
 * @author ZZT 
 * @brief 
 *        
 * @version 0.1
 * @date 2025-5-8
 * 
 * @copyright Copyright (c) 2025
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "Encoder.h"


static uint8_t Encoder_Get_Data(uint8_t *data,pub_Encoder_Data *Encoder_Data);

static uint8_t Encoder_Rtos_Init(Encoder_Instance_t *Encoder_instance,uint32_t queue_length);


uint8_t Encoder_rx_buffer[100];


uart_package_t Encoder_uart_package = {
    .uart_handle = &huart1, 
    .rx_buffer = Encoder_rx_buffer,
    .rx_buffer_size = 20,
    .uart_callback = Encoder_Uart_Rx_Callback,
};

union Encoder
{
    uint8_t data[4];
    uint32_t   Encoder_conut;
}Encoder_data;


Encoder_Instance_t* Encoder_init(Uart_Instance_t *Encoder_uart_instance,uint32_t queue_length)
{
    if(Encoder_uart_instance == NULL)
    {
        LOGERROR("Encoder init failed!");
        return NULL;
    }
    Encoder_Instance_t *temp_Encoder_instance = (Encoder_Instance_t *)pvPortMalloc(sizeof(Encoder_Instance_t));
    if(temp_Encoder_instance == NULL)
    {
        LOGERROR("Encoder instance malloc failed!");
        return NULL;
    }
    memset(temp_Encoder_instance,0,sizeof(Encoder_Instance_t));
    
    temp_Encoder_instance->Encoder_uart_instance = Encoder_uart_instance;
    
    temp_Encoder_instance->Encoder_pub = register_pub("Encoder_pub");
    if(temp_Encoder_instance->Encoder_pub == NULL)
    {
        LOGERROR("Encoder pub register failed!");
        vPortFree(temp_Encoder_instance);
        temp_Encoder_instance = NULL;
        return NULL;
    }
    
    if(Encoder_Rtos_Init(temp_Encoder_instance,queue_length) != 1)
    {
        LOGERROR("Encoder rtos init failed!");
        vPortFree(temp_Encoder_instance);
        temp_Encoder_instance = NULL;
        return NULL;
    }

    temp_Encoder_instance->get_data = Encoder_Get_Data;
    temp_Encoder_instance->Encoder_task = Encoder_Task;
    temp_Encoder_instance->Encoder_deinit = Encoder_DeInit;

    temp_Encoder_instance->Encoder_uart_instance->device = temp_Encoder_instance;
    LOGINFO("Encoder instance init success!");
    return temp_Encoder_instance;
}   


/**
 * @brief 
 * 
 * @param Encoder_instance 
 * @param queue_length 
 * @return uint8_t 
 */
static uint8_t Encoder_Rtos_Init(Encoder_Instance_t *Encoder_instance,uint32_t queue_length)
{
    if(Encoder_instance == NULL)
    {
        LOGERROR("Encoder instance is NULL!");
        return 0;
    }
    rtos_for_module_t *rtos_for_Encoder = (rtos_for_module_t *)pvPortMalloc(sizeof(rtos_for_module_t));
    if(rtos_for_Encoder == NULL)
    {
        LOGERROR("rtos_for_Encoder malloc failed!");
        return 0;
    }
    memset(rtos_for_Encoder,0,sizeof(rtos_for_module_t));

    rtos_for_Encoder->queue_send = queue_send_wrapper;
    rtos_for_Encoder->queue_receive = xQueueReceive;

    QueueHandle_t queue = xQueueCreate(queue_length,sizeof(UART_TxMsg));
    if(queue == NULL)
    {
        LOGERROR("queue create failed!");
        vPortFree(rtos_for_Encoder);
        rtos_for_Encoder = NULL;
        return 0;
    }
    else
    {
        rtos_for_Encoder->xQueue = queue;
    }
    Encoder_instance->rtos_for_Encoder = rtos_for_Encoder;
    LOGINFO("Encoder rtos init success!");
    return 1;
}


uint8_t Encoder_Task(void* Encoder_instance)
{
    UART_TxMsg Msg;
    if(Encoder_instance == NULL)
    {
        LOGERROR("Encoder instance is NULL!");
        return 0;
    }
    Encoder_Instance_t *temp_Encoder_instance = Encoder_instance;
    if(temp_Encoder_instance->rtos_for_Encoder->xQueue != NULL && temp_Encoder_instance->rtos_for_Encoder->queue_receive != NULL)
    {
        if(temp_Encoder_instance->rtos_for_Encoder->queue_receive(temp_Encoder_instance->rtos_for_Encoder->xQueue,&Msg,0) == pdPASS)
        {
            if(Encoder_Get_Data(Msg.data_addr,&temp_Encoder_instance->pub_data) == 1)
            {
                publish_data temp_data;
                temp_data.data = (uint8_t*)&temp_Encoder_instance->pub_data;
                temp_data.len = sizeof(pub_Encoder_Data);
                temp_Encoder_instance->Encoder_pub->publish(temp_Encoder_instance->Encoder_pub,temp_data);
                return 1;
            }
        }
    }
    return 0;
}


uint8_t Encoder_Get_Data(uint8_t *data,pub_Encoder_Data *Encoder_Data)
{
    if(data == NULL)
    {
        LOGERROR("Encoder instance is NULL!");
        return 0;
    }
    if(data[0] != 0xAB || data[1] != 0xCD)
    {
        LOGERROR("header is wrong!");
        return 0;
    }
    uint8_t data_len = 0;
    data_len = data[2];
    if(10 != data_len)
    {

    }
    Encoder_data.data[0] = data[4];
    Encoder_data.data[1] = data[3];
    Encoder_data.data[2] = data[6];
    Encoder_data.data[3] = data[5];
    // if(data[7] != 0xD5 || data[8] != 0xD6)
    // {
    //     LOGERROR("tail is wrong!");
    //     return 0;
    // }

    Encoder_Data->distance = Encoder_data.Encoder_conut/4096*0.01;
    memset(data,0,10);    
    return 1;
}


uint8_t Encoder_Uart_Rx_Callback(void *uart_instance,uint16_t data_len)
{
    UART_TxMsg Msg;
    if(uart_instance == NULL)
    {
        LOGERROR("uart_instance is NULL!");
        return 0;
    }
    Uart_Instance_t *temp_uart_instance = (Uart_Instance_t*)uart_instance;
    Encoder_Instance_t *temp_Encoder_instance = temp_uart_instance->device;

    if(temp_Encoder_instance->rtos_for_Encoder->xQueue != NULL && temp_Encoder_instance->rtos_for_Encoder->queue_send != NULL)
    {
        Msg.data_addr = temp_Encoder_instance->Encoder_uart_instance->uart_package.rx_buffer;
        Msg.len = data_len;
        Msg.huart = temp_Encoder_instance->Encoder_uart_instance->uart_package.uart_handle;
        if(Msg.data_addr != NULL)
        {
            temp_Encoder_instance->rtos_for_Encoder->queue_send(temp_Encoder_instance->rtos_for_Encoder->xQueue,&Msg,NULL);
            return 1;
        }
    }

    return 0;
}


uint8_t Encoder_DeInit(void *Encoder_instance)
{
    if(Encoder_instance == NULL)
    {
        LOGERROR("Encoder instance is NULL!");
        return 0;
    }
    Encoder_Instance_t *temp_Encoder_instance = Encoder_instance;

    
    if(temp_Encoder_instance->rtos_for_Encoder != NULL)
    {
        temp_Encoder_instance->rtos_for_Encoder->queue_receive = NULL;
        temp_Encoder_instance->rtos_for_Encoder->queue_send = NULL;
        if(temp_Encoder_instance->rtos_for_Encoder->xQueue != NULL)
        {
            vQueueDelete(temp_Encoder_instance->rtos_for_Encoder->xQueue);
            temp_Encoder_instance->rtos_for_Encoder->xQueue = NULL;
        }
        vPortFree(temp_Encoder_instance->rtos_for_Encoder);
        temp_Encoder_instance->rtos_for_Encoder = NULL;
    }

    
    if(temp_Encoder_instance->Encoder_uart_instance != NULL)
    {
        temp_Encoder_instance->Encoder_uart_instance->Uart_Deinit(temp_Encoder_instance->Encoder_uart_instance);
    }

    temp_Encoder_instance->get_data = NULL;
    temp_Encoder_instance->Encoder_task = NULL;
    temp_Encoder_instance->Encoder_deinit = NULL;

    vPortFree(temp_Encoder_instance);
    temp_Encoder_instance = NULL;
    Encoder_instance = NULL;
    return 1;
}