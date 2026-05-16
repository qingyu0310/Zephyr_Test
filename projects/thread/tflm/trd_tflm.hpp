/**
 * @file trd_tflm.hpp
 * @author qingyu
 * @brief TFLM Hello World inference thread
 * @version 0.1
 * @date 2026-05-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "stdint.h"

namespace thread::ml {
    void thread_init();
    void thread_start(uint8_t prio = 10);
};
