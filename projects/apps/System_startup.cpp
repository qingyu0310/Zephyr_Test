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

void mcan4_rx_callback_func(struct can_frame &frame, void *)
{
    using namespace instance::chassis;

    switch (frame.id)
    {
        case kSteerCanId[0]:
            chassis_wheel[0].steer_motor.CanCpltRxCallback(frame.data);
            break;

        case kDriveCanId[0]:
            chassis_wheel[0].drive_motor.CanCpltRxCallback(frame.data);
            break;
            
        case kSteerCanId[1]:
            chassis_wheel[1].steer_motor.CanCpltRxCallback(frame.data);
            break;

        case kDriveCanId[1]:
            chassis_wheel[1].drive_motor.CanCpltRxCallback(frame.data);
            break;
            
        #if USE_POWERMETER
        case KSteerPwrMeterId:
            SteerPwrMeter.CanCpltRxCallback(frame.data);
            break;
        case KDrivePwrMeterId:
            DrivePwrMeter.CanCpltRxCallback(frame.data);
            break;
        #endif

        default:
            break;
    }
}

using namespace thread;

void System_Bsp_Init()
{
    can::thread_init();
}

void System_Modules_Init()
{
    output::thread_init();
    chassis::thread_init();
}

void System_Thread_Start()
{
    can::thread_start();
    output::thread_start();
    chassis::thread_start();
}
