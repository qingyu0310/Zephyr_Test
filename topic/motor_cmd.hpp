#pragma once

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

enum CanIndex : uint16_t
{ 
    CAN1 = 0,
    CAN2,
    CAN3,
    CAN4,
    CAN5,
};

struct motor_cmd_msg {
    uint16_t can_id;
    uint16_t tx_id;
    uint8_t data[8];

    static constexpr float kCurrentScale    = 16384.0f / 20.0f; ///< A → DJI raw
    static constexpr float kCurrentScaleInv = 20.0f / 16384.0f; ///< DJI raw → A

    /** 设置电机电流（A），内部转 DJI 原始值，高位在前 */
    void SetOut(uint8_t idx, float current_A)
    {
        int16_t raw = static_cast<int16_t>(current_A * kCurrentScale);
        data[idx * 2 + 0] = static_cast<uint8_t>(raw >> 8);
        data[idx * 2 + 1] = static_cast<uint8_t>(raw & 0xFF);
    }

    /** 读取电机电流（A） */
    float GetOut(uint8_t idx) const
    {
        int16_t raw = static_cast<int16_t>(
            (static_cast<uint16_t>(data[idx * 2 + 0]) << 8) |
             static_cast<uint16_t>(data[idx * 2 + 1])
        );
        return static_cast<float>(raw) * kCurrentScaleInv;
    }
};

ZBUS_CHAN_DECLARE(motor_cmd_chan);
ZBUS_OBS_DECLARE(can_tx_sub);
