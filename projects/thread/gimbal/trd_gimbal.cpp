/**
 * @file trd_gimbal.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "trd_gimbal.hpp"
#include "dm.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "to_can_tx.hpp"
#include "remote_to.hpp"
#include "zephyr/kernel.h"
#include "zephyr/zbus/zbus.h"

namespace thread::gimbal {

using namespace instance::gimbal;

static Thread<> thread_{};

constexpr uint8_t kTaskRunningCycle = 1;                                    // 任务运行周期（ms）
constexpr float   kYawRadPerCycle   = 3.0f * kTaskRunningCycle * 0.001f;    // yaw   每周期弧度 0.001为 ms 转换到 s 的常数
constexpr float   kPitchRadPerCycle = 2.0f * kTaskRunningCycle * 0.001f;    // pitch 每周期弧度 0.001为 ms 转换到 s 的常数

// static float g_syaw_radian  = 0.0f;
static float g_byaw_radian  = 0.0f;
static float g_pitch_radian = 0.0f;

static void Task(void*, void*, void*)
{
    topic::to_can_tx::Message msg{};

    for (;;)
    {
        const zbus_channel *chan = nullptr;
        zbus_sub_wait(&sub_remote_to, &chan, K_NO_WAIT);
        if (chan) {
            topic::remote_to::Message rx{};
            zbus_chan_read(chan, &rx, K_NO_WAIT);
            g_byaw_radian  += rx.yaw   * kYawRadPerCycle;
            g_pitch_radian += rx.pitch * kPitchRadPerCycle;
        }

        {
            const auto snap = byaw_.ReadAll();

            switch (snap.err) 
            {
                case DmErrorStatus::Enable:
                {
                    // 外环：角度 → 角速度
                    byaw_ctrl_.position.SetTarget(g_byaw_radian);
                    byaw_ctrl_.position.SetNow(snap.radian);
                    const float omega_ref = byaw_ctrl_.position.CalcAngle();

                    // 内环：角速度 → 转矩
                    byaw_ctrl_.omega.SetTarget(omega_ref);
                    byaw_ctrl_.omega.SetNow(snap.omega);
                    const float tor_ref = byaw_ctrl_.omega.Calc();

                    byaw_.SetTargetRad(g_byaw_radian);
                    byaw_.SetTargetTor(tor_ref);
                    byaw_.CtrlData(msg.data);
                    msg.tx_id = byaw_.GetTxId();

                    k_msgq_put(&user_can1_msgq, &msg, K_NO_WAIT);

                    break;
                }
                case DmErrorStatus::Disable:
                {
                    msg.tx_id = byaw_.GetTxId();
                    byaw_.EnableData(msg.data);

                    k_msgq_put(&user_can1_msgq, &msg, K_NO_WAIT);

                    break;
                }
                default:
                    break;
            }
        }

        k_msleep(1);
    }
}

void thread_init()
{
    {
        alg::pid::Pid::Config position_cfg {};
        position_cfg.kp  = 1.0f;
        position_cfg.ki  = 0.0f;
        position_cfg.kd  = 0.0f;

        alg::pid::Pid::Config omega_cfg{};
        omega_cfg.kp = 0.2f;
        omega_cfg.ki = 0.0f;
        omega_cfg.kd = 0.0f;

        DmMotor::Config cfg {
            .ctrl_met       = ControlMethon::Mit,
            .can_id         = 0x01,
            .master_id      = 0x00,
            .gearbox_ratio  = 1.0f,
            .wheel_r        = 1,
            .kp             = 0.0f,
            .kd             = 0.0f,
            .pos_max        = 12.5,
            .vel_max        = 45,
            .tor_max        = 10,
        };
        
        byaw_.Init(cfg);
        byaw_ctrl_.position.Init(position_cfg);
        byaw_ctrl_.omega.Init(omega_cfg);
    }
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

}










































