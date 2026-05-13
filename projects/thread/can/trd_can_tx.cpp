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
#include <zephyr/drivers/gpio.h>
#include "chassis_to_can.hpp"
#include "thread.hpp"
#include <string.h>
#include <zephyr/zbus/zbus.h>

namespace thread::can {

static Thread<> thread_{};
static const zbus_channel *chan = nullptr;
static topic::chassis_to_can::Message msg{};
static can_frame tx_frame{};

static Can can1{};

static void Task(void*, void*, void*)
{
    for (;;)
    {
        zbus_sub_wait(&sub_chassis_to_can, &chan, K_FOREVER);
        zbus_chan_read(chan, &msg, K_NO_WAIT);

        tx_frame.id  = msg.tx_id;
        tx_frame.dlc = 8;
        memcpy(tx_frame.data, msg.data, 8);

        can1.Send(&tx_frame);
    }
}

void thread_init()
{
    {
        const gpio_dt_spec stby = { .port = DEVICE_DT_GET(DT_NODELABEL(gpiof)), .pin = 2, .dt_flags = 0 };
        if (device_is_ready(stby.port)) {
            gpio_pin_configure_dt(&stby, GPIO_OUTPUT_LOW);
        }
        const device* can_dev   = DEVICE_DT_GET(DT_ALIAS(user_can1));
        if (!device_is_ready(can_dev)) {
            return;
        }
        const can_filter filter {
            .id    = 0,
            .mask  = 0,
            .flags = 0,
        };
        can1.Init(can_dev, filter);
        can1.SetRxCallback(user_can1_rx_callback);
    }
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}


} // namespace thread::can
