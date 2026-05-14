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
#include "to_can_tx.hpp"
#include "thread.hpp"
#include <string.h>

namespace thread::can {

static Thread<> thread_{};
static Can user_can1{};

static void Task(void*, void*, void*)
{
    can_frame tx{};
    topic::to_can_tx::Message msg{};

    for (;;)
    {
        k_msgq_get(&user_can1_msgq, &msg, K_FOREVER);
        tx.id  = msg.tx_id;
        tx.dlc = 8;
        memcpy(tx.data, msg.data, 8);
        user_can1.Send(&tx);
    }
}

void thread_init()
{
    {
        const gpio_dt_spec stby = { .port = DEVICE_DT_GET(DT_NODELABEL(gpiof)), .pin = 2, .dt_flags = 0 };
        if (device_is_ready(stby.port)) {
            gpio_pin_configure_dt(&stby, GPIO_OUTPUT_LOW);
        }
        const device* dev = DEVICE_DT_GET(DT_ALIAS(user_can1));
        if (!device_is_ready(dev)) return;
        const can_filter filter { .id = 0, .mask = 0, .flags = 0 };
        user_can1.Init(dev, filter);
        user_can1.SetRxCallback(user_can1_rx_callback);
    }
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}


} // namespace thread::can
