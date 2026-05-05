/**
 * @file chassis_to_can.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/zbus/zbus.h>
#include "chassis_to_can.hpp"

ZBUS_CHAN_DEFINE(pub_chassis_to_can,
                 topic::chassis_to_can::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(sub_chassis_to_can),
                 ZBUS_MSG_INIT({}));

ZBUS_SUBSCRIBER_DEFINE(sub_chassis_to_can, 10);
