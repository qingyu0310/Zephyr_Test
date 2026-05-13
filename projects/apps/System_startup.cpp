/**
 * @file System_startup.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "System_startup.h"
#include "trd_chassis.hpp"
#include "trd_can_tx.hpp"
#include "trd_gpio.hpp"
#include "remote.hpp"

using namespace thread;

void System_Bsp_Init()
{
    can    ::thread_init();
}

void System_Modules_Init()
{
    output ::thread_init();
    chassis::thread_init();
    remote ::thread_init();
}

void System_Thread_Start()
{
    can    ::thread_start();
    chassis::thread_start();
    output ::thread_start();
    remote ::thread_start();
}
