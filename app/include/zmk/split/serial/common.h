/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/types.h>
#include <init.h>

/* Caller does/should not need to free `data`
 * Data will be freed immediately after calling this callback */
typedef int (*rx_complete_t)(const uint8_t *data, size_t length);

void split_serial_async_init(rx_complete_t complete_fn);

void split_serial_async_send(uint8_t *data, size_t length);

uint8_t *alloc_split_serial_buffer(k_timeout_t timeout);

void free_split_serial_buffer(const uint8_t *data);

#define SPLIT_DATA_LEN 16

#define SPLIT_TYPE_KEYPOSITION 0

typedef struct _split_data_t {
    uint16_t type;
    uint8_t data[SPLIT_DATA_LEN];
    uint16_t crc;
} split_data_t;

int zmk_split_position_pressed(uint8_t position);

int zmk_split_position_released(uint8_t position);
