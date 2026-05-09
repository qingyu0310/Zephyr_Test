#include <zephyr/zbus/zbus.h>
#include "remote_to.hpp"

ZBUS_CHAN_DEFINE(pub_remote_to,
                 topic::remote_to::RemoteData,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(sub_remote_to),
                 ZBUS_MSG_INIT({}));

ZBUS_SUBSCRIBER_DEFINE(sub_remote_to, 10);
