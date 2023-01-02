#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <zmk/behavior.h>

int zmk_split_invoke_behavior(uint8_t source, struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event, bool state);

#define ZMK_SPLIT_IS_CENTRAL                                                                       \
    (IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

#define ZMK_SPLIT_PERIPHERAL_COUNT 1
