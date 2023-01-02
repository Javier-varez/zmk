/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zmk/split/serial/common.h>
#include <init.h>

#include <sys/crc.h>
#include <device.h>
#include <drivers/uart.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/matrix.h>
#include <zmk/stdlib.h>

K_MSGQ_DEFINE(notify_event_msgq, sizeof(struct zmk_position_state_msg),
              CONFIG_ZMK_SPLIT_SERIAL_THREAD_QUEUE_SIZE, 4);
K_THREAD_STACK_DEFINE(notify_q_stack, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE);
struct k_work_q notify_work_q;

static void split_central_notify_func(struct k_work *work) {
    struct zmk_inter_kb_msg split_data = {};
    k_msgq_get(&notify_event_msgq, &split_data, K_FOREVER);

    static uint8_t position_state[ZMK_MAX_BITMAP_LEN];
    uint8_t changed_positions[ZMK_MAX_BITMAP_LEN];

    LOG_INF("[NOTIFICATION] type:%u", split_data.type);
    if (split_data.type != ZmkKbMsgTypePositionState) {
        return;
    }

    for (int i = 0; i < ZMK_MAX_BITMAP_LEN; i++) {
        changed_positions[i] = split_data.position_state_msg.bitmap[i] ^ position_state[i];
        position_state[i] = split_data.position_state_msg.bitmap[i];
    }

    for (int i = 0; i < ZMK_MAX_BITMAP_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (changed_positions[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = position_state[i] & BIT(j);
                struct zmk_position_state_changed ev = {
                    .position = position, .state = pressed, .timestamp = k_uptime_get()};

                if (position > ZMK_KEYMAP_LEN) {
                    LOG_WRN("Invalid position: %u", position);
                    continue;
                }

                LOG_INF("Trigger key position state change for %d", ev.position);
                ZMK_EVENT_RAISE(new_zmk_position_state_changed(ev));
            }
        }
    }

    return;
}

K_WORK_DEFINE(notify_work, split_central_notify_func);

// This runs on ISR context
static void split_serial_rx_callback(const struct zmk_inter_kb_msg *msg) {
    int retval = 0;
    if ((retval = k_msgq_put(&notify_event_msgq, msg, K_NO_WAIT)) < 0) {
        LOG_ERR("Unable to queue serial split message: %d", retval);
        return;
    }

    if ((retval = k_work_submit_to_queue(&notify_work_q, &notify_work)) < 0) {
        LOG_ERR("Unable to submit split serial work item: %d", retval);
        return;
    }
}

static int split_serial_init() {
    zmk_split_serial_init(split_serial_rx_callback);
    k_work_queue_start(&notify_work_q, notify_q_stack, K_THREAD_STACK_SIZEOF(notify_q_stack),
                       CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY, NULL);
    return 0;
}

int zmk_split_invoke_behavior(uint8_t source, struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event, bool state) {
    struct zmk_inter_kb_msg payload = {
        .type = ZmkKbMsgTypeBehavior,
        .crc = 0,
        .behavior_msg =
            {
                .position = event.position,
                .state = state ? 1 : 0,
                .param1 = binding->param1,
                .param2 = binding->param2,
            },
    };

    const size_t payload_dev_size = sizeof(payload.behavior_msg.behavior_dev);

    if (strlcpy(payload.behavior_msg.behavior_dev, binding->behavior_dev, payload_dev_size) >=
        payload_dev_size) {
        LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                log_strdup(binding->behavior_dev), log_strdup(payload.behavior_msg.behavior_dev));
    }

    LOG_DBG("Sending behavior to dev \"%s\" event pos %d, state %d, param1 %d, param2 %d",
            payload.behavior_msg.behavior_dev, payload.behavior_msg.position,
            payload.behavior_msg.state, payload.behavior_msg.param1, payload.behavior_msg.param2);
    zmk_split_serial_send(&payload);
    return 0;
}

SYS_INIT(split_serial_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
