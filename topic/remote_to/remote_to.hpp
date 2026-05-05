#pragma once

#include <zephyr/zbus/zbus.h>
#include "dr16.hpp"

namespace topic::remote_to {

using DR16Message = thread::remote::DR16Output;

} // namespace topic::remote_to

ZBUS_CHAN_DECLARE(pub_dr16_to);
ZBUS_OBS_DECLARE(sub_dr16_to);
