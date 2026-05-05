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
#include "remote_to.hpp"
#include "thread.hpp"
#include "chassis_to_can.hpp"
#include "power_ctrl.hpp"
#include "pid.hpp"
#include "zephyr/zbus/zbus.h"
#if USE_POWERMETER
#include "zephyr/sys/printk.h"
#include "debug.hpp"
#endif

namespace {

constexpr float kPi   = 3.1415926535f;
constexpr float kPi_2 = kPi / 2.f;
constexpr float k2Pi  = 2.f * kPi;

float NormalizeAngle(float a)
{
    a = fmodf(a, k2Pi);
    if (a >  kPi) a -= k2Pi;
    if (a < -kPi) a += k2Pi;
    return a;
}

}

namespace thread::chassis {

using namespace instance::chassis;

static Thread<> thread_{};                                              // chassis线程

static constexpr float   kTorqueK        = 0.3f;                        // C6xx 转矩常数 N·m/A
static constexpr uint8_t kTotalBudget    = 20;                          // 底盘总功率预算 W

struct { float x, y; } static constexpr kWheelPos[N_Wheel] {
    {0.0f,  0.135f},                                          // 轮0: 前 (y = +R)
    {0.0f, -0.135f},                                          // 轮1: 后 (y = -R)
};
static constexpr float KMaxMoveVelocity  = 2.0f;                        // 最大移动线速度
static constexpr float KMaxRotationOmega = 2.0f;                        // 最大旋转角速度 (rad/s)

static constexpr int8_t  kAngleSign[N_Wheel]  = {-1, -1};       // 舵向角度方向补偿 [前, 后]
static constexpr int8_t  kSwerveSign[N_Wheel] = {-1, -1};       // 行进速度方向补偿 [前, 后]

static alg::power_ctrl::PowerCtrl SteerPwrCtrl{};                       // 转向电机功率控制器
static alg::power_ctrl::PowerCtrl DrivePwrCtrl{};                       // 行进电机功率控制器

struct {
    float angle;                                                        // 目标舵向角 rad
    float velocity;                                                     // 目标行进速度 m/s
} static g_wh_target[N_Wheel] {};                                       // 运动学输出
      
static float g_k_factor[N_Wheel]    {};                                 // 优劣弧方向因子
static float g_steer_target[N_Wheel] {};                                 // 优劣弧调整后角度
static float g_vx = 0.0f, g_vy = 0.0f, g_vw = 0.0f;                     // 底盘速度指令

static topic::chassis_to_can::Message msg_chassis_to_can {};            // topic 发送
static const zbus_channel* chan = nullptr;                              // topic 接收
static topic::remote_to::DR16Message msg_remote_to_chassis{};           // topic 接收

/**
 * @brief N轮舵轮逆向运动学解算：V_wheel = V_chassis + ω × r_wheel
 */
static void SwerveKinCalculate()
{
    for (uint8_t wi = 0; wi < N_Wheel; wi++)
    {
        const float vx_w = g_vx - g_vw * kWheelPos[wi].y;
        const float vy_w = g_vy + g_vw * kWheelPos[wi].x;
        const float spd = sqrtf(vx_w * vx_w + vy_w * vy_w);

        if (spd > 1e-6f) {
            g_wh_target[wi].angle = kAngleSign[wi] * atan2f(vx_w, vy_w);
            g_wh_target[wi].angle = NormalizeAngle(g_wh_target[wi].angle);
        }

        g_wh_target[wi].velocity = spd * kSwerveSign[wi];
    }
}

/**
 * @brief 优劣弧判断：对比当前角度与目标角度，必要时翻转目标并反转行进方向
 * @param current 当前底盘角度 (rad)
 * @param target 目标角度 (rad)，若走劣弧则修改为翻转后的目标
 * @return 方向因子：1.0 正转 / -1.0 反转
 */
static float OptimalArc(float current, float& target)
{
    float err = target - current;
    err = NormalizeAngle(err);

    if (fabsf(err) > kPi_2) {
        // 走短路径：目标角度 ±π，行进电机反转
        target = NormalizeAngle(current + err - (err > 0.f ? kPi : -kPi));
        return -1.f;
    }
    return 1.f;
}

/**
 * @brief 读取遥控器数据并转换为底盘速度指令
 */
inline static void ReadRemote()
{
    zbus_sub_wait(&sub_dr16_to, &chan, K_NO_WAIT);
    if (chan) {
        zbus_chan_read(chan, &msg_remote_to_chassis, K_NO_WAIT);
        g_vx = msg_remote_to_chassis.axis.x * KMaxMoveVelocity;
        g_vy = msg_remote_to_chassis.axis.y * KMaxMoveVelocity;
        g_vw = msg_remote_to_chassis.axis.yaw * KMaxRotationOmega;
    }
}

/**
 * @brief 运动学解算 + 优劣弧调整，更新所有电机的目标值
 */
inline static void UpdateTarget()
{
    SwerveKinCalculate();

    for (uint8_t wi = 0; wi < N_Wheel; wi++)
    {
        g_k_factor[wi] = 1.f;
        g_steer_target[wi] = g_wh_target[wi].angle;

        const auto snap = chassis_wheel[wi].steer_motor.ReadAll();
        const float chassis_angle = NormalizeAngle(-snap.angle);

        g_k_factor[wi] = OptimalArc(chassis_angle, g_steer_target[wi]);
    }
}

/**
 * @brief 所有电机 PID 串联计算（外环角度/速度 → 内环力矩 → 电流）
 */
inline static void ControlCalculate()
{
    for (uint8_t wi = 0; wi < N_Wheel; wi++)
    {
        // 转向：角度 → 力矩串联
        {
            const auto snap = chassis_wheel[wi].steer_motor.ReadAll();
            const float chassis_angle = NormalizeAngle(-snap.angle);

            wheel_pid[wi].steer_angle.SetTarget(g_steer_target[wi]);
            wheel_pid[wi].steer_angle.SetNow(chassis_angle);
            const float torque_ref = wheel_pid[wi].steer_angle.CalcAngle();
            const float current_ref = wheel_pid[wi].steer_torque.Calc(torque_ref, snap.torque) / kTorqueK;
            SteerPwrCtrl.SetTarget(wi, current_ref);
            SteerPwrCtrl.SetMotorData(wi, snap.torque, snap.omega, wheel_pid[wi].steer_angle.GetError());
        }

        // 行进：速度 → 力矩串联 (speed × K)
        {
            const auto snap = chassis_wheel[wi].drive_motor.ReadAll();
            const float target = g_wh_target[wi].velocity * g_k_factor[wi];

            wheel_pid[wi].drive_velocity.SetTarget(target);
            wheel_pid[wi].drive_velocity.SetNow(snap.velocity);
            const float torque_ref  = wheel_pid[wi].drive_velocity.Calc();
            const float current_ref = wheel_pid[wi].drive_torque.Calc(torque_ref, snap.torque) / kTorqueK;
            DrivePwrCtrl.SetTarget(wi, current_ref);
            DrivePwrCtrl.SetMotorData(wi, snap.torque, snap.omega, wheel_pid[wi].drive_velocity.GetError());
        }
    }
}

/**
 * @brief 功率预测 + 功率分配（转向组优先，行进组分剩余）
 */
inline static void PowerAlloc()
{
    // 功率预测 + 分配
    #if USE_POWERMETER
    SteerPwrCtrl.SetMeasuredPower(SteerPwrMeter.GetPower());
    DrivePwrCtrl.SetMeasuredPower(DrivePwrMeter.GetPower());
    #endif
    SteerPwrCtrl.Predict();
    DrivePwrCtrl.Predict();

    {
        constexpr float kTurnRatio = 0.8f;                          // 转向电机分配功率比例
        constexpr float turnMax = kTotalBudget * kTurnRatio;        // 转向电机允许分配最大功率比例

        const float turnPred = SteerPwrCtrl.GetTotalPower();
        SteerPwrCtrl.Allocate(turnMax);

        const float runBudget = kTotalBudget - ((turnPred < turnMax) ? turnPred : turnMax);
        DrivePwrCtrl.Allocate(runBudget);
    }
}

/**
 * @brief 组帧并发布到 CAN 发送 topic
 */
inline static void FramePublish()
{
    // 组帧发布 (数据索引查表 kSteerDataIdx/kDriveDataIdx)
    for (uint8_t wi = 0; wi < N_Wheel; wi++)
    {
        msg_chassis_to_can.SetOut(kSteerDataIdx[wi], SteerPwrCtrl.GetLimitedCurrent(wi));
        msg_chassis_to_can.SetOut(kDriveDataIdx[wi], DrivePwrCtrl.GetLimitedCurrent(wi));
    }

    zbus_chan_pub(&pub_chassis_to_can, &msg_chassis_to_can, K_MSEC(1));
}


/**
 * @brief 底盘控制主循环 遥控读取 → 运动学 → PID → 功率 → 发布
 */
static void Task(void*, void*, void*)
{
    static constexpr uint32_t kPeriodMs = 1; // 控制周期 ms

    for (;;)
    {
        const int64_t tick_start = k_uptime_get();

        ReadRemote();
        UpdateTarget();
        ControlCalculate();
        PowerAlloc();
        FramePublish();

        #if USE_POWERMETER
        if (debug::chassis::pwr_run) {
        printk("run,%e,%e,%f,%f\n", static_cast<double>(DrivePwrCtrl.GetK1()), static_cast<double>(DrivePwrCtrl.GetK2()),
                    static_cast<double>(DrivePwrMeter.GetPower()), static_cast<double>(DrivePwrCtrl.GetTotalPower()));
        }
        if (debug::chassis::pwr_turn) {
            printk("turn,%e,%e,%f,%f\n", static_cast<double>(SteerPwrCtrl.GetK1()), static_cast<double>(SteerPwrCtrl.GetK2()),
                        static_cast<double>(SteerPwrMeter.GetPower()), static_cast<double>(SteerPwrCtrl.GetTotalPower()));
        }
        #endif

        const int64_t elapsed = k_uptime_get() - tick_start;
        const int64_t remain = static_cast<int64_t>(kPeriodMs) - elapsed;
        if (remain > 0) {
            k_msleep(remain);
        }
    }
}

void thread_init()
{
    // 功率计初始化
    #if USE_POWERMETER
    {
        SteerPwrMeter.Init(KSteerPwrMeterId);
        DrivePwrMeter.Init(KDrivePwrMeterId);
    }
    #endif

    // 功率预测模型初始化
    {
        constexpr float kSteerk1Init = 1.453009e-07;
        constexpr float kSteerk2Init = 5.171939e-03;
        constexpr float KSteerk3Init = 3.0f;

        // 转向组：上限 80%，角度误差敏感
        {
            alg::power_ctrl::PowerCtrl::Config cfg{};
            cfg.k1Init      = kSteerk1Init;
            cfg.k2Init      = kSteerk2Init;
            cfg.torqueK     = kTorqueK;
            cfg.motorCount  = N_Wheel;
            cfg.k3          = KSteerk3Init;
            cfg.errUpper    = 50.0f;
            cfg.errLower    = 0.01f;
            cfg.rlsLambda   = 0.999f;
            #if USE_POWERMETER
            cfg.rlsEnable   = true;
            #else
            cfg.rlsEnable   = false;
            #endif
            cfg.tauOmegaEnable = cfg.rlsEnable;

            SteerPwrCtrl.Init(cfg);
        }

        constexpr float kDrivek1Init = 1.453009e-07;
        constexpr float kDrivek2Init = 5.171939e-03;
        constexpr float KDrivek3Init = 3.5f;

        // 行进组：拿剩余，速度误差范围大
        {
            alg::power_ctrl::PowerCtrl::Config cfg{};
            cfg.k1Init      = kDrivek1Init;
            cfg.k2Init      = kDrivek2Init;
            cfg.torqueK     = kTorqueK;
            cfg.motorCount  = N_Wheel;
            cfg.k3          = KDrivek3Init;
            cfg.errUpper    = 500.0f;
            cfg.errLower    = 0.001f;
            #if USE_POWERMETER
            cfg.rlsEnable   = true;
            cfg.rlsLambda   = 0.999f;
            #else
            cfg.rlsEnable   = false;
            #endif
            cfg.tauOmegaEnable = cfg.rlsEnable;

            DrivePwrCtrl.Init(cfg);
        }
    }

    // 电机 + PID 统一初始化（每轮先转向后行进）
    for (uint8_t wi = 0; wi < N_Wheel; wi++)
    {
        // 转向电机：底盘 + 外环角度 + 内环力矩
        {
            constexpr float kWheelR       = 0.1f;
            constexpr float kGearboxRatio = 3591.f / 187.f;

            DjiC6xx::Config motor_cfg{};
            motor_cfg.rx_id         = kSteerCanId[wi];
            motor_cfg.wheel_r       = kWheelR;
            motor_cfg.gearbox_ratio = kGearboxRatio;

            alg::pid::Pid::Config angle_pid_cfg{};
            angle_pid_cfg.kp  = 1.0f;
            angle_pid_cfg.ki  = 0.0f;
            angle_pid_cfg.kd  = 0.0f;

            alg::pid::Pid::Config torque_pid_cfg{};
            torque_pid_cfg.kp = 0.2f;
            torque_pid_cfg.ki = 0.0f;
            torque_pid_cfg.kd = 0.0f;

            chassis_wheel[wi].steer_motor.Init(motor_cfg);
            wheel_pid[wi].steer_angle.    Init(angle_pid_cfg);
            wheel_pid[wi].steer_torque.   Init(torque_pid_cfg);
        }

        // 行进电机：底盘 + 外环速度 + 内环力矩
        {
            constexpr float kWheelR       = 0.1f;
            constexpr float kGearboxRatio = 3591.f / 187.f;
            
            DjiC6xx::Config motor_cfg {};
            motor_cfg.rx_id         = kDriveCanId[wi];
            motor_cfg.wheel_r       = kWheelR;
            motor_cfg.gearbox_ratio = kGearboxRatio;

            alg::pid::Pid::Config speed_pid_cfg{};
            speed_pid_cfg.kp  = 5.0f;
            speed_pid_cfg.ki  = 0.0f;
            speed_pid_cfg.kd  = 0.0f;

            alg::pid::Pid::Config torque_pid_cfg{};
            torque_pid_cfg.kp = 0.5f;
            torque_pid_cfg.ki = 0.0f;
            torque_pid_cfg.kd = 0.0f;

            chassis_wheel[wi].drive_motor.Init(motor_cfg);
            wheel_pid[wi].drive_velocity. Init(speed_pid_cfg);
            wheel_pid[wi].drive_torque.   Init(torque_pid_cfg);
        }
    }
}

void thread_start(uint8_t prio, void* p2, void* p3)
{
    thread_.Start(Task, prio);
}

} // namespace thread::chassis
