/**
 * @file bsp_flash.h
 * @author zzt 
 * @brief 
 * @version 0.1
 * @date 2025-7-4
 * 
 * @copyright Copyright (c) 2025
 * @note :
 * @versioninfo :
 */
#ifndef BSP_FLASH_H 
#define BSP_FLASH_H 

#ifdef __cplusplus
extern "C"{
#endif
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx_hal.h"
#include <string.h>
#include "bsp_log.h"
/*-----------------------------------macro------------------------------------*/

 #define FLASH_SAVE_ADDR  0x080E0000  // 注意：必须保证这个区域没有程序代码
//#define FLASH_SAVE_ADDR  0x08020000  // 注意：必须保证这个区域没有程序代码
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------function----------------------------------*/
HAL_StatusTypeDef Flash_SaveArray(uint32_t *data, uint32_t length);
void Flash_LoadArray(uint32_t *dest, uint32_t length);


#ifdef __cplusplus
}
#endif

#endif	/* BSP_FLASH_H */



