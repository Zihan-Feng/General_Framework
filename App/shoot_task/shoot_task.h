/**
 * @file shoot_task.h
 * @author STHY
 * @brief
 * @version 0.1
 * @date 2025-5-13
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "queue.h"


/*-----------------------------------macro------------------------------------*/
#define ARRAY_SIZE       8
/*----------------------------------typedef-----------------------------------*/
typedef struct {
    uint32_t shoot_flash_buf[ARRAY_SIZE];
    uint8_t  Flash_Mark;
}Flash_Data_t;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void Shoot_Task(void *argument);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "Unitree_Go1.h"
#include "VESC_motor.h"
#include "chassis_task.h"
#include "com_config.h"
#include "motor_interface.h"
#include "rm_motor.h"
#include "DM_motor.h"
 #include "flash.h"

#endif
