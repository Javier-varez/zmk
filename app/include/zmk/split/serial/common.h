/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/types.h>
#include <init.h>

void split_serial_sync_init();
void split_serial_sync_send(const uint8_t *const data, const size_t length);
void split_serial_sync_recv(uint8_t *const data, const size_t length);

#define SPLIT_DATA_LEN 16

#define SPLIT_TYPE_KEYPOSITION 0

typedef struct _split_data_t {
    uint16_t type;
    uint8_t data[SPLIT_DATA_LEN];
    uint16_t crc;
} split_data_t;

#define ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN 9

struct zmk_split_run_behavior_data {
    uint8_t position;
    uint8_t state;
    uint32_t param1;
    uint32_t param2;
} __packed;

struct zmk_split_run_behavior_payload {
    struct zmk_split_run_behavior_data data;
    char behavior_dev[ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN];
} __packed;

int zmk_split_position_pressed(uint8_t position);
int zmk_split_position_released(uint8_t position);
