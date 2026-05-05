/**
 * @file remote_topic.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/zbus/zbus.h>
#include "remote_to.hpp"

ZBUS_CHAN_DEFINE(pub_dr16_to,
                 topic::remote_to::DR16Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(sub_dr16_to),
                 ZBUS_MSG_INIT({0}));

ZBUS_SUBSCRIBER_DEFINE(sub_dr16_to, 10);

