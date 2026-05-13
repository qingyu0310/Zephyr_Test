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

#include "stdint.h"

namespace thread::output {
    void thread_init();
    void thread_start(uint8_t prio = 5);
};

namespace thread::input {
    void thread_init();
    void thread_start(uint8_t prio = 5);
};
