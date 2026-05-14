/**
 * @file to_can_tx.hpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

extern struct k_msgq user_can1_msgq;

namespace topic::to_can_tx {

struct Message {
    uint16_t tx_id;
    uint8_t  data[8];
};

constexpr auto *chassis = &user_can1_msgq;    // 底盘 CAN（换总线只改这里）
constexpr auto *gimbal  = &user_can1_msgq;    // 云台 CAN

} // namespace topic::to_can_tx
