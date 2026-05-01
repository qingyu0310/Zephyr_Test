/**
 * @file Init.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Init.h"
#include "System_startup.h"
#include <zephyr/kernel.h>

void Init()
{
   System_Bsp_Init();
   System_Modules_Init();
   System_Thread_Start();
}














