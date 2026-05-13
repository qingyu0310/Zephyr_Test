/**
 * @file trd_gpio.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_gpio.hpp"
#include "thread.hpp"
#include "output.hpp"
#include "input.hpp"
#include "timer.hpp"

#define GPIO_GET(node_id)   GPIO_DT_SPEC_GET(DT_NODELABEL(node_id), gpios)

namespace thread::output {

static Thread<> thread_{};
static Output led_r{};

static void Task(void*, void*, void*)
{
    Timer timer_(1000);

    for (;;)
    {
        timer_.Update();

        timer_.Clock([&]{
            led_r.Toggle();
        });

        k_msleep(1);
    }
}

void thread_init()
{
    led_r.init(GPIO_GET(led_alert));
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

} // namespace thread::output

namespace thread::input {

static Thread<> thread_{};

/*  占位：预留用于按键/传感器输入，待实现  */
static void Task(void*, void*, void*)
{
    for (;;)
    {
        k_msleep(100);
    }
}

void thread_init()
{
    
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}


} // namespace thread::input
