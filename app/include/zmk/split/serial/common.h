/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/types.h>
#include <init.h>

#define ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN 9

// Max bitmap size is a 8 x 16 matrix
#define ZMK_MAX_BITMAP_LEN (8 * 16 / 8)

enum zmk_kb_msg_type {
    ZmkKbMsgTypeBehavior = 0,
    ZmkKbMsgTypePositionState = 1,
};

struct zmk_position_state_msg {
    uint8_t bitmap[ZMK_MAX_BITMAP_LEN];
} __packed;

struct zmk_behavior_split_msg {
    uint8_t position;
    uint8_t state;
    uint32_t param1;
    uint32_t param2;
    char behavior_dev[ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN];
} __packed;

struct zmk_inter_kb_msg {
    enum zmk_kb_msg_type type;
    uint16_t crc;
    union {
        struct zmk_position_state_msg position_state_msg;
        struct zmk_behavior_split_msg behavior_msg;
    };
} __packed;

typedef void (*zmk_split_serial_rx_callback_t)(const uint8_t *data, const int len);

/**
 * \brief Initializes the serial inter-split communication.
 * \param rx_callback A callback that is called from the ISR context whenever a packet has been
 *                    received. CRC is not validated beforehand.
 */
int zmk_split_serial_sync_init(zmk_split_serial_rx_callback_t rx_callback);

/**
 * \brief Sends the data through the serial transport
 * \param msg A pointer to the message to be sent.
 */
int zmk_split_serial_send(const struct zmk_inter_kb_msg *msg);

/**
 * \brief Invoked by ZMK when a key is pressed
 */
int zmk_split_position_pressed(uint8_t position);

/**
 * \brief Invoked by ZMK when a key is released
 */
int zmk_split_position_released(uint8_t position);
