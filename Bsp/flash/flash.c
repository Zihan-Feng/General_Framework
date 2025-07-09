#include "flash.h"
#include "stm32f4xx_hal_flash.h" // 假设你在使用STM32F4，根据你的芯片型号修改

// 假设你的LOGINFO宏或函数已经定义，例如：
// #define LOGINFO(...)  printf(__VA_ARGS__)
// 或者是一个更复杂的日志系统
// #include "my_log_system.h" // 引入你的日志系统头文件

// 假设 FLASH_SAVE_ADDR 和 FLASH_SECTOR_11 已经正确定义
// #define FLASH_SAVE_ADDR    (FLASH_BASE + 0x0E0000) // 示例，请根据实际扇区11起始地址定义
// #define FLASH_SECTOR_11    FLASH_SECTOR_11_BASE_ADDR // 示例

HAL_StatusTypeDef Flash_SaveArray(uint32_t *data, uint32_t length)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t SectorError = 0; // 用于存储擦除失败时的扇区错误信息

    // 1. 解锁闪存
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        LOGINFO("Error: Flash Unlock Failed! Status: %d\r\n", status);
        return status; // 返回解锁失败的错误状态
    }
    LOGINFO("Flash Unlocked successfully.\r\n");

    // 2. 擦除扇区 11
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 电压范围 2.7V - 3.6V
    EraseInitStruct.Sector       = FLASH_SECTOR_11;       // 确保这个宏定义正确对应扇区11的起始地址
    EraseInitStruct.NbSectors    = 1;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK)
    {
        LOGERROR("Error: Flash Erase Failed! Status: %d, SectorError: 0x%lX\r\n", status, SectorError);
        HAL_FLASH_Lock(); // 擦除失败也要锁定闪存
        return status; // 返回擦除失败的错误状态
    }
    LOGINFO("Flash Sector 11 Erased successfully.\r\n");

    // 3. 顺序写入每个 32 位数据
    for (uint32_t i = 0; i < length; i++)
    {
        // 确保写入地址在正确的扇区范围内，并且不超过扇区大小
        // 例如，如果FLASH_SAVE_ADDR是扇区11的起始，且扇区大小是128KB
        // 那么 i * 4 应该小于 128KB
        // 请确保这里的 FLASH_SECTOR_11_SIZE_BYTES 宏已经正确定义为你芯片扇区11的实际大小
        // 例如：#define FLASH_SECTOR_11_SIZE_BYTES (128U * 1024U) // 128KB
        if ((FLASH_SAVE_ADDR + i * 4) >= (0x08000000+1024*1024))
        {
            LOGERROR("Warning: Attempting to write beyond allocated flash range for sector 11. Aborting write.\r\n");
            status = HAL_ERROR; // 或者其他自定义错误码
            break; // 跳出循环，避免越界写入
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   FLASH_SAVE_ADDR + i * 4,
                                   data[i]);
        if (status != HAL_OK)
        {
            LOGERROR("Error: Flash Program Failed at address 0x%lX! Status: %d\r\n",
                   (FLASH_SAVE_ADDR + i * 4), status);
            HAL_FLASH_Lock(); // 编程失败也要锁定闪存
            return status; // 返回编程失败的错误状态
        }
    }

    // 4. 检查循环是否因为错误而退出
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock(); // 确保闪存被锁定
        return status;    // 返回写入失败的错误状态
    }

    HAL_FLASH_Lock(); // 所有操作成功，锁定闪存
    LOGINFO("Flash Array Saved successfully.\r\n");
    return HAL_OK;
}
void Flash_LoadArray(uint32_t *dest, uint32_t length)
{
    memcpy(dest, (uint32_t *)FLASH_SAVE_ADDR, length * 4);
}
