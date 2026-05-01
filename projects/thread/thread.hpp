/**
 * @file thread.hpp
 * @author qingyu
 * @brief Zephyr 线程模板，封装线程创建、启动等样板代码
 * @version 0.1
 * @date 2026-04-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <zephyr/kernel.h>
#include <cstdint>

/**
 * @brief 线程模板，封装 k_thread 创建和初始化
 *
 * @tparam StackSize 线程栈大小
 */
template<uint32_t StackSize = 1024>
class Thread final
{
public:
    using init = void (*)();

    void Start(k_thread_entry_t entry, int prio = 5, void* p1 = nullptr, void* p2 = nullptr, void* p3 = nullptr)
    {
        k_thread_create(&thread_, stack_, K_THREAD_STACK_SIZEOF(stack_),
                        entry, p1, p2, p3, prio, 0, K_NO_WAIT);
    }

private:
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, StackSize);
};

/**
 * @brief 一个简单的线程类，仅在 Start 时创建线程
 *
 * @tparam StackSize
 */
template<uint32_t StackSize = 1024>
class SimpleThread final
{
public:
    void Start(k_thread_entry_t entry, int prio = 5, void* p1 = nullptr, void* p2 = nullptr, void* p3 = nullptr)
    {
        k_thread_create(&thread_, stack_, K_THREAD_STACK_SIZEOF(stack_),
                        entry, p1, p2, p3, prio, 0, K_NO_WAIT);
    }

private:
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, StackSize);
};

#define THREAD_INIT(Entry, Thread, ...) \
    Thread.Init(Entry, ##__VA_ARGS__)
