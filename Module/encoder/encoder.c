/**
 * @file Encoder.c
 * @author ZZT，STHY
 * @brief 
 *        
 * @version 0.2
 * @date 2025-6-24
 * 
 * @copyright Copyright (c) 2025
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "Encoder.h"

#define ENCODER_BITS   18
#define ENCODER_MAX    ((1 << ENCODER_BITS) - 1)
#define ENCODER_HALF   (1 << (ENCODER_BITS - 1))

static uint8_t Encoder_Get_Data(uint8_t *data,pub_Encoder_Data *Encoder_Data);

static uint8_t Encoder_Rtos_Init(Encoder_Instance_t *Encoder_instance,uint32_t queue_length);

static int32_t encoder_delta(uint32_t prev, uint32_t curr);
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
int32_t Encoder_count_ = 0;

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


uint8_t Encoder_Get_Data(uint8_t *data, pub_Encoder_Data *Encoder_Data)
{
    static uint32_t Encoder_last = 0;
    static uint32_t Encoder_curt = 0;
    static int32_t  delta = 0;

    if (data == NULL) {
        LOGERROR("Encoder instance is NULL!");
        return 0;
    }

    // 1. 检查功能码
    if (data[1] != 0x03) {
        LOGERROR("Modbus function code error!");
        return 0;
    }

    // 2. 获取字节数
    uint8_t byte_count = data[2];
    if (byte_count < 4) { 
        LOGERROR("Modbus data length error!");
        return 0;
    }

    // 3. CRC16校验
    uint16_t crc_recv = (data[3 + byte_count+ 1] << 8) | data[3 + byte_count ];
    uint16_t crc_calc = CRC16_Table(data, 3 + byte_count); // 此计算函数低位在前，高位在后！！！！
    if (crc_recv != crc_calc) {
        LOGWARNING("Modbus CRC error!");
        return 0;
    }

    // 4. 解析编码器数据（高字节在前，低字节在后）
    Encoder_curt = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6];

    delta = encoder_delta(Encoder_last, Encoder_curt);
    Encoder_Data->distance -= (float)delta / 4096 * 0.01;
    
    Encoder_last = Encoder_curt;

    return 1;
}

/**
 * @brief 发送写0x0007寄存器命令（自动回传时间）
 * @param uart_handle UART句柄
 * @param addr 从站地址
 * @param value 自动回传时间（0~65535，单位ms）
 * @return 0失败，1成功
 */
uint8_t Encoder_Send_Write_0x0007(UART_HandleTypeDef *uart_handle, uint8_t addr, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = addr;
    frame[1] = 0x06;
    frame[2] = 0x00;                // 寄存器高字节
    frame[3] = 0x07;                // 寄存器低字节
    frame[4] = (value >> 8) & 0xFF; // 数值高字节
    frame[5] = value & 0xFF;        // 数值低字节
    uint16_t crc = CRC16_Table(frame, 6);
    frame[6] = (crc >> 8) & 0xFF;   // CRC高字节
    frame[7] = crc & 0xFF;          // CRC低字节

    // 发送帧
    if(HAL_UART_Transmit(uart_handle, frame, 8, 100) == HAL_OK)
        return 1;
    else
        return 0;
}

/**
 * @brief 解析写0x0007寄存器的回传响应
 * @param data 回传数据指针
 * @param len  数据长度
 * @return 0失败，1成功
 */
uint8_t Encoder_Parse_Write_0x0007_Response(uint8_t *data, uint16_t len)
{
    if(len != 8) return 0;
    // 功能码、寄存器地址、数值等应与发送一致
    if(data[1] != 0x06) return 0;
    if(data[2] != 0x00 || data[3] != 0x07) return 0;
    // CRC校验
    uint16_t crc_recv = (data[6] << 8) | data[7];
    uint16_t crc_calc = CRC16_Table(data, 6);
    if(crc_recv != crc_calc) return 0;
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
static int32_t encoder_delta(uint32_t prev, uint32_t curr) {
    int32_t delta = (int32_t)(curr - prev);
    if (delta > ENCODER_HALF) {
        delta -= ENCODER_MAX + 1;  // Wrap around
    } else if (delta < -ENCODER_HALF) {
        delta += ENCODER_MAX + 1;   // Wrap around
    }
    return delta;
}