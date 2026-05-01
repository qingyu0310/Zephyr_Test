/**
 * @file trd_chassis.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_chassis.hpp"
#include "motor_cmd.hpp"
#include "power_ctrl.hpp"
#include "pid.hpp"
#include "zephyr/sys/printk.h"

using namespace instance::chassis;

namespace thread::chassis {

constexpr float k3508_k1Init = 1.453009e-07;
constexpr float k3508_k2Init = 5.171939e-03;

static alg::power_ctrl::PowerCtrl pwrCtrl;

static void Task(void*, void*, void*)
{
    motor_cmd_msg msg{};
    msg.can_id = CanIndex::CAN1;
    msg.tx_id = 0x200;

    for (;;)
    {
        auto snap = motors1.ReadAll();
        const float rawOut = pidCtrl.Calc(0.5f, snap.torque);

        pwrCtrl.SetTarget(0, rawOut);
        pwrCtrl.SetMotorData(0, snap.torque, snap.omega, pidCtrl.GetError());
        #if USE_POWERMETER
        float power = meter1.GetPower();
        pwrCtrl.SetMeasuredPower(power);
        #endif
        pwrCtrl.Predict();
        pwrCtrl.Allocate(20);

        msg.SetOut(0, pwrCtrl.GetLimitedCurrent(0));

        zbus_chan_pub(&motor_cmd_chan, &msg, K_MSEC(1));

        if (dbg_enabled) {
            printk("%e,%e,%f\n", pwrCtrl.GetK1(), pwrCtrl.GetK2(), pwrCtrl.GetTotalPower());
        }

        k_msleep(2);
    }
}

void thread_init()
{
    {
        alg::power_ctrl::PowerCtrl::Config pwr_cfg{};
        pwr_cfg.k1Init = k3508_k1Init;
        pwr_cfg.k2Init = k3508_k2Init;
        pwr_cfg.motorCount = 1;
        pwr_cfg.tauOmegaEnable = false;
        pwr_cfg.rlsEnable = true;

        pwrCtrl.Init(pwr_cfg);
    }
    
    {
        alg::pid::Pid::Config pid_cfg{};
        pid_cfg.kp = 1.0f;
        pid_cfg.ki = 0.0f;
        pid_cfg.kd = 0.0f;

        pidCtrl.Init(pid_cfg);
    }
    
    #if USE_POWERMETER
    {   
        meter1.Init(0x002);
    }
    #endif
    
    {
        DjiC6xx::Config cfg {};
        cfg.rx_id = 0x201;

        motors1.Init(cfg);
    }
}

void thread_start(uint8_t prio, void* p2, void* p3)
{
    thread_.Start(Task, prio);
}

}
