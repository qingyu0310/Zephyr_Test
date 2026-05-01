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

#include "thread.hpp"
#include "dji_c6xx.hpp"
#include "powermeter.hpp"
#include "pid.hpp"

#define USE_POWERMETER      0

namespace thread::chassis {
    inline Thread<> thread_{};
    void thread_init();
    void thread_start(uint8_t prio = 5, void* p2 = nullptr, void* p3 = nullptr);
};

namespace instance::chassis {
#if USE_POWERMETER
    inline PowerMeter meter1{};
#endif
    inline DjiC6xx motors1{};
    inline alg::pid::Pid pidCtrl{};
    inline bool dbg_enabled = false;
}