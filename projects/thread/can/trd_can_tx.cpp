/**
 * @file trd_can_tx.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_can_tx.hpp"
#include <string.h>
#include <zephyr/zbus/zbus.h>
#include "motor_cmd.hpp"

using namespace instance::can;

namespace thread::can {


static void Task(void*, void*, void*)
{
    const zbus_channel *chan;
    motor_cmd_msg msg;
    can_frame tx_frame;

    for (;;)
    {
        zbus_sub_wait(&can_tx_sub, &chan, K_FOREVER);
        zbus_chan_read(chan, &msg, K_NO_WAIT);

        (void)memset(&tx_frame, 0, sizeof(tx_frame));
        tx_frame.id  = msg.tx_id;
        tx_frame.dlc = 8;
        (void)memcpy(tx_frame.data, msg.data, 8);

        mcan4.Send(&tx_frame);

        k_msleep(2);
    }
}

void thread_init()
{
    {
        const gpio_dt_spec stby = { .port = DEVICE_DT_GET(DT_NODELABEL(gpioz)), .pin = 2, .dt_flags = 0 };
        const device* can_dev = DEVICE_DT_GET(DT_NODELABEL(mcan4));
        if (!device_is_ready(can_dev)) {
            return;
        }
        const can_filter filter = {
            .id   = 0,
            .mask = 0,
            .flags = 0,
        };
        mcan4.Init(can_dev, filter, CAN_MODE_NORMAL, &stby);
        mcan4.SetRxCallback(mcan4_rx_callback_func);
    }
}

void thread_start(uint8_t prio, void* p2, void* p3)
{
    thread_.Start(Task, prio, nullptr, p2, p3);
}


} // namespace thread::can
