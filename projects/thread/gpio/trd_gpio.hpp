/**
 * @file trd_gpio.h
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "thread.hpp"

namespace thread::output {
    inline Thread<> thread_{};
    void thread_init();
    void thread_start(uint8_t prio = 5, void* p2 = nullptr, void* p3 = nullptr);
};

namespace thread::input {
    inline Thread<> thread_{};
    void thread_init();
    void thread_start(uint8_t prio = 5, void* p2 = nullptr, void* p3 = nullptr);
};
