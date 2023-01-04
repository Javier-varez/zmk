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

static uint8_t position_state[ZMK_MAX_BITMAP_LEN];

K_THREAD_STACK_DEFINE(position_state_q_stack, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE);

struct k_work_q position_state_work_q;

K_MSGQ_DEFINE(position_state_msgq, sizeof(struct zmk_position_state_msg),
              CONFIG_ZMK_SPLIT_SERIAL_THREAD_QUEUE_SIZE, 4);

static void send_position_state_callback(struct k_work *work) {
    struct zmk_inter_kb_msg position_state_msg;

    memset(&position_state_msg, 0, sizeof(struct zmk_inter_kb_msg));
    position_state_msg.type = ZmkKbMsgTypePositionState;
    position_state_msg.crc = 0;

    while (k_msgq_get(&position_state_msgq, &position_state_msg.position_state_msg, K_NO_WAIT) ==
           0) {
        zmk_split_serial_send(&position_state_msg);
    }
};

K_WORK_DEFINE(service_position_notify_work, send_position_state_callback);

static int send_position_state() {
    int err = k_msgq_put(&position_state_msgq, position_state, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Position state message queue full, popping first message and queueing again");
            struct zmk_position_state_msg discarded_state;
            k_msgq_get(&position_state_msgq, &discarded_state, K_NO_WAIT);
            return send_position_state();
        }
        default:
            LOG_WRN("Failed to queue position state to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&position_state_work_q, &service_position_notify_work);
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

K_MSGQ_DEFINE(inter_kbd_msgq, sizeof(struct zmk_inter_kb_msg),
              CONFIG_ZMK_SPLIT_SERIAL_THREAD_QUEUE_SIZE, 4);
K_THREAD_STACK_DEFINE(inter_kbd_q_stack, CONFIG_ZMK_SPLIT_SERIAL_THREAD_STACK_SIZE);
struct k_work_q inter_kbd_work_q;

static void split_svc_run_behavior(struct k_work *work) {
    struct zmk_inter_kb_msg payload = {};
    k_msgq_get(&inter_kbd_msgq, &payload, K_FOREVER);

    // TODO(javier-varez): Validate CRC

    if (payload.type != ZmkKbMsgTypeBehavior) {
        LOG_ERR("Invalid message type received: %d", payload.type);
        return;
    }

    struct zmk_behavior_binding binding = {
        .param1 = payload.behavior_msg.param1,
        .param2 = payload.behavior_msg.param2,
        .behavior_dev = payload.behavior_msg.behavior_dev,
    };
    LOG_DBG("%s with params %d %d: pressed? %d", log_strdup(binding.behavior_dev), binding.param1,
            binding.param2, payload.behavior_msg.state);
    const struct zmk_behavior_binding_event event = {.position = payload.behavior_msg.position,
                                                     .timestamp = k_uptime_get()};
    int err;
    if (payload.behavior_msg.state > 0) {
        err = behavior_keymap_binding_pressed(&binding, event);
    } else {
        err = behavior_keymap_binding_released(&binding, event);
    }

    if (err) {
        LOG_ERR("Failed to invoke behavior %s: %d", log_strdup(binding.behavior_dev), err);
    }
}

K_WORK_DEFINE(behavior_work, split_svc_run_behavior);

// This runs on ISR context
static void split_serial_rx_callback(const uint8_t *data, const int len) {
    struct zmk_inter_kb_msg *inter_kb_data = (struct zmk_inter_kb_msg *)data;
    if (len > sizeof(struct zmk_inter_kb_msg)) {
        LOG_ERR("Exceeded maximum size for serial split message: %d", len);
        return;
    }

    int retval = 0;
    if ((retval = k_msgq_put(&inter_kbd_msgq, inter_kb_data, K_NO_WAIT)) < 0) {
        LOG_ERR("Unable to queue serial split message: %d", retval);
        return;
    }

    if ((retval = k_work_submit_to_queue(&inter_kbd_work_q, &behavior_work)) < 0) {
        LOG_ERR("Unable to submit split serial work item: %d", retval);
        return;
    }
}

static int split_serial_init() {
    k_work_queue_start(&inter_kbd_work_q, inter_kbd_q_stack,
                       K_THREAD_STACK_SIZEOF(inter_kbd_q_stack),
                       CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY, NULL);

    k_work_queue_start(&position_state_work_q, position_state_q_stack,
                       K_THREAD_STACK_SIZEOF(position_state_q_stack),
                       CONFIG_ZMK_SPLIT_SERIAL_THREAD_PRIORITY, NULL);

    zmk_split_serial_sync_init(split_serial_rx_callback);
    return 0;
}

SYS_INIT(split_serial_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
