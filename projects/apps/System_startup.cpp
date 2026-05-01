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
#include "trd_gpio.hpp"
#include "trd_can_tx.hpp"

void mcan4_rx_callback_func(struct can_frame &frame, void *)
{
    switch (frame.id)
    {
        #if USE_POWERMETER
        case (0x002):
        {
            instance::chassis::meter1.CanCpltRxCallback(frame.data);
            break;
        }
        #endif
        case (0x201):
        {
            instance::chassis::motors1.CanCpltRxCallback(frame.data);
            break;
        }
        default:
            // printk("%d\n", frame.id);
            break;
    }
}

void System_Bsp_Init()
{

}

void System_Modules_Init()
{
    thread::output::thread_init();
    thread::chassis::thread_init();
    thread::can::thread_init();
}

void System_Thread_Start()
{
    thread::output::thread_start();
    thread::chassis::thread_start();
    thread::can::thread_start();
}
