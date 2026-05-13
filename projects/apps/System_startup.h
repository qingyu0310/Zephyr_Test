/**
 * @file System_startup.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif 

void System_Bsp_Init();
void System_Modules_Init();
void System_Thread_Start();

#ifdef __cplusplus
}
#endif



