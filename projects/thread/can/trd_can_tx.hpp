/**
 * @file trd_can_tx.hpp
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
#include "can.hpp"

void mcan4_rx_callback_func(struct can_frame &frame, void *);

namespace thread::can {
    inline Thread<> thread_{};
    void thread_init();
    void thread_start(uint8_t prio = 5, void* p2 = nullptr, void* p3 = nullptr);
};

namespace instance::can {
    inline Can mcan4{};
}


