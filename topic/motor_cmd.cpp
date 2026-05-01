/**
 * @file chan_def.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <zephyr/zbus/zbus.h>
#include "motor_cmd.hpp"

ZBUS_CHAN_DEFINE(motor_cmd_chan,
                motor_cmd_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(can_tx_sub),
                 ZBUS_MSG_INIT({0}));

ZBUS_SUBSCRIBER_DEFINE(can_tx_sub, 10);
