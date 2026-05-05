/**
 * @file trd_chassis.hpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "dji_c6xx.hpp"
#include "pid.hpp"
#include <cstdint>

#define USE_POWERMETER      0

#if USE_POWERMETER
#include "powermeter.hpp"
#endif

namespace thread::chassis {
    void thread_init();
    void thread_start(uint8_t prio = 5, void* p2 = nullptr, void* p3 = nullptr);
};

namespace instance::chassis {

    #if USE_POWERMETER
    constexpr uint16_t KSteerPwrMeterId = 0x01;
    constexpr uint16_t KDrivePwrMeterId = 0x02;
    inline PowerMeter  SteerPwrMeter {};
    inline PowerMeter  DrivePwrMeter {};
    #endif

    constexpr uint8_t  N_Wheel = 2;

    constexpr uint16_t kSteerCanId[N_Wheel]   = {0x202, 0x204};
    constexpr uint16_t kDriveCanId[N_Wheel]   = {0x201, 0x203};
    constexpr uint8_t  kDriveDataIdx[N_Wheel] = {static_cast<uint8_t>(kDriveCanId[0] - 0x201), static_cast<uint8_t>(kDriveCanId[1] - 0x201)};
    constexpr uint8_t  kSteerDataIdx[N_Wheel] = {static_cast<uint8_t>(kSteerCanId[0] - 0x201), static_cast<uint8_t>(kSteerCanId[1] - 0x201)};

    struct WheelModule {
        DjiC6xx steer_motor{};
        DjiC6xx drive_motor{};
    };

    struct WheelPid {
        alg::pid::Pid steer_angle{};
        alg::pid::Pid steer_torque{};
        alg::pid::Pid drive_velocity{};
        alg::pid::Pid drive_torque{};
    };

    inline WheelPid wheel_pid[N_Wheel] {};
    inline WheelModule chassis_wheel[N_Wheel] {};
}