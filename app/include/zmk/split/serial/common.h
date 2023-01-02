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

int zmk_split_position_pressed(uint8_t position);
int zmk_split_position_released(uint8_t position);
