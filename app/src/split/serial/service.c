/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zmk/split/split.h>
#include <zmk/split/serial/common.h>
#include <sys/util.h>
#include <sys/types.h>
#include <sys/crc.h>
#include <init.h>

#include <device.h>
#include <drivers/behavior.h>
#include <drivers/uart.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/matrix.h>

static uint8_t position_state[SPLIT_DATA_LEN];

K_THREAD_STACK_DEFINE(service_q_stack, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE);

struct k_work_q service_work_q;

K_MSGQ_DEFINE(position_state_msgq, sizeof(char[SPLIT_DATA_LEN]),
              CONFIG_ZMK_SPLIT_SERIAL_THREAD_QUEUE_SIZE, 4);

static void send_position_state_callback(struct k_work *work) {
    split_data_t split_data;

    memset(&split_data, 0, sizeof(split_data_t));
    split_data.type = SPLIT_TYPE_KEYPOSITION;

    while (k_msgq_get(&position_state_msgq, split_data.data, K_NO_WAIT) == 0) {
        split_data.crc = crc16_ansi(split_data.data, sizeof(split_data.data));
        split_serial_sync_send((const uint8_t *)&split_data, sizeof(split_data));
    }
};

K_WORK_DEFINE(service_position_notify_work, send_position_state_callback);

static int send_position_state() {
    int err = k_msgq_put(&position_state_msgq, position_state, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Position state message queue full, popping first message and queueing again");
            uint8_t discarded_state[SPLIT_DATA_LEN];
            k_msgq_get(&position_state_msgq, &discarded_state, K_NO_WAIT);
            return send_position_state();
        }
        default:
            LOG_WRN("Failed to queue position state to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&service_work_q, &service_position_notify_work);
    return 0;
}

int zmk_split_position_pressed(uint8_t position) {
    WRITE_BIT(position_state[position / 8], position % 8, true);
    return send_position_state();
}

int zmk_split_position_released(uint8_t position) {
    WRITE_BIT(position_state[position / 8], position % 8, false);
    return send_position_state();
}

K_MSGQ_DEFINE(behavior_event_msgq, sizeof(struct zmk_split_run_behavior_payload),
              CONFIG_ZMK_SPLIT_SERIAL_THREAD_QUEUE_SIZE, 8);
K_THREAD_STACK_DEFINE(behavior_q_stack, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE);
struct k_work_q behavior_work_q;

static void split_svc_run_behavior(struct k_work *work) {
    struct zmk_split_run_behavior_payload payload = {};
    k_msgq_get(&behavior_event_msgq, &payload, K_FOREVER);

    struct zmk_behavior_binding binding = {
        .param1 = payload.data.param1,
        .param2 = payload.data.param2,
        .behavior_dev = payload.behavior_dev,
    };
    LOG_DBG("%s with params %d %d: pressed? %d", log_strdup(binding.behavior_dev), binding.param1,
            binding.param2, payload.data.state);
    const struct zmk_behavior_binding_event event = {.position = payload.data.position,
                                                     .timestamp = k_uptime_get()};
    int err;
    if (payload.data.state > 0) {
        err = behavior_keymap_binding_pressed(&binding, event);
    } else {
        err = behavior_keymap_binding_released(&binding, event);
    }

    if (err) {
        LOG_ERR("Failed to invoke behavior %s: %d", log_strdup(binding.behavior_dev), err);
    }
}

K_WORK_DEFINE(behavior_work, split_svc_run_behavior);

static int split_serial_rx_thread() {
    k_work_queue_start(&behavior_work_q, behavior_q_stack, K_THREAD_STACK_SIZEOF(behavior_q_stack),
                       CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY, NULL);

    k_work_queue_start(&service_work_q, service_q_stack, K_THREAD_STACK_SIZEOF(service_q_stack),
                       CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY, NULL);

    split_serial_sync_init();

    // Seems like an extra byte is detected on boot, so we should discard it... Obviously this is
    // not quite reliable and a more sophisticated mechanism should be used (like using framing for
    // data packets).
    uint8_t data;
    split_serial_sync_recv((uint8_t *)&data, sizeof(data));

    k_sleep(K_MSEC(10));

    while (true) {
        struct zmk_split_run_behavior_payload data;

        memset(&data, 0, sizeof(data));
        split_serial_sync_recv((uint8_t *)&data, sizeof(data));

        if (k_msgq_put(&behavior_event_msgq, &data, K_NO_WAIT) < 0) {
            LOG_ERR("Lost behavior event");
            continue;
        }
        k_work_submit_to_queue(&behavior_work_q, &behavior_work);
    }

    return 0;
}

K_THREAD_DEFINE(split_service, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE, split_serial_rx_thread, 0,
                0, 0, /*CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY*/ 0, 0, 0);
