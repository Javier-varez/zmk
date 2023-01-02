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

K_MSGQ_DEFINE(notify_event_msgq, sizeof(split_data_t), CONFIG_ZMK_SPLIT_SERIAL_THREAD_QUEUE_SIZE,
              8);
K_THREAD_STACK_DEFINE(notify_q_stack, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE);
struct k_work_q notify_work_q;

static void split_central_notify_func(struct k_work *work) {
    split_data_t split_data = {};
    k_msgq_get(&notify_event_msgq, &split_data, K_FOREVER);

    static uint8_t position_state[SPLIT_DATA_LEN];
    uint8_t changed_positions[SPLIT_DATA_LEN];
    uint16_t crc;

    LOG_INF("[NOTIFICATION] type:%u CRC:%u", split_data.type, split_data.crc);

    crc = crc16_ansi(split_data.data, sizeof(split_data.data));
    if (crc != split_data.crc) {
        LOG_WRN("CRC mismatch (%x:%x), skipping data", crc, split_data.crc);
        return;
    }

    for (int i = 0; i < SPLIT_DATA_LEN; i++) {
        changed_positions[i] = split_data.data[i] ^ position_state[i];
        position_state[i] = split_data.data[i];
    }

    for (int i = 0; i < SPLIT_DATA_LEN; i++) {
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

                LOG_DBG("Trigger key position state change for %d", ev.position);
                ZMK_EVENT_RAISE(new_zmk_position_state_changed(ev));
            }
        }
    }

    return;
}

K_WORK_DEFINE(notify_work, split_central_notify_func);

static int split_serial_rx_thread() {
    split_serial_sync_init();
    k_work_queue_start(&notify_work_q, notify_q_stack, K_THREAD_STACK_SIZEOF(notify_q_stack),
                       CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY, NULL);

    while (true) {
        split_data_t data;
        split_serial_sync_recv((uint8_t *)&data, sizeof(data));

        k_msgq_put(&notify_event_msgq, &data, K_NO_WAIT);
        k_work_submit_to_queue(&notify_work_q, &notify_work);
    }

    return 0;
}

K_THREAD_DEFINE(split_central, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE, split_serial_rx_thread, 0,
                0, 0, /*CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY*/ 0, 0, 0);

int zmk_split_invoke_behavior(uint8_t source, struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event, bool state) {
    struct zmk_split_run_behavior_payload payload = {
        .data =
            {
                .param1 = binding->param1,
                .param2 = binding->param2,
                .position = event.position,
                .state = state ? 1 : 0,
            },
    };

    const size_t payload_dev_size = sizeof(payload.behavior_dev);

    if (strlcpy(payload.behavior_dev, binding->behavior_dev, payload_dev_size) >=
        payload_dev_size) {
        LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                log_strdup(binding->behavior_dev), log_strdup(payload.behavior_dev));
    }

    LOG_DBG("Sending behavior to dev \"%s\" event pos %d, state %d, param1 %d, param2 %d",
            payload.behavior_dev, payload.data.position, payload.data.state, payload.data.param1,
            payload.data.param2);
    split_serial_sync_send((const uint8_t *)&payload, sizeof(payload));
    return 0;
}
