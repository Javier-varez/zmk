/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdbool.h>
#include <zmk/split/serial/common.h>
#include <init.h>

#include <sys/crc.h>
#include <device.h>
#include <drivers/uart.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/matrix.h>

#if !DT_HAS_CHOSEN(zmk_split_serial)
#error "No zmk,split-serial node is chosen"
#endif

#define UART_NODE1 DT_CHOSEN(zmk_split_serial)
#define MAX_COBS_MSG_LEN 256

K_SEM_DEFINE(tx_done, 0, 1);

typedef struct cobs_state {
    uint8_t next_zero;
} cobs_state_t;

typedef struct line_data {
    uint8_t *ptr;
    uint32_t len;
    uint32_t off;
    atomic_t in_progress;
} line_data_t;

typedef struct uart_data {
    const struct device *const serial_dev;
    bool ready;
    zmk_split_serial_rx_callback_t rx_callback;

    line_data_t tx;
    line_data_t rx;
    cobs_state_t rx_cobs_state;
} uart_data_t;

static uart_data_t uart_data = {.ready = false, .serial_dev = DEVICE_DT_GET(UART_NODE1)};

// Returns true if the buffer is now complete with the current packet
static int uart_handle_rxed_byte(line_data_t *const rx, cobs_state_t *const cobs_state,
                                 const uint8_t byte) {
    if (cobs_state->next_zero == 0) {
        // First byte in the packet
        cobs_state->next_zero = byte;
        return 0;
    }

    if (--cobs_state->next_zero == 0) {
        if (byte == 0) {
            // End of frame marker
            return 1;
        }

        // Virtual zero
        rx->ptr[rx->off++] = 0;
        cobs_state->next_zero = byte;
        return 0;
    }

    if (byte == 0) {
        // This looks like a framing error...
        rx->off = 0;
        cobs_state->next_zero = 0;
        return -EIO;
    }

    // Regular data
    rx->ptr[rx->off++] = byte;
    return 0;
}

static void uart_irq_callback_user_data(const struct device *const dev, void *const user_data) {
    uart_data_t *const data = (uart_data_t *)user_data;
    if (!uart_irq_update(dev)) {
        return;
    }

    if (atomic_get(&data->tx.in_progress) != 0) {
        line_data_t *const tx = &data->tx;
        while ((tx->off < tx->len) && uart_irq_tx_ready(dev)) {
            const int nbytes = uart_fifo_fill(dev, &tx->ptr[tx->off], tx->len - tx->off);
            if (nbytes < 0) {
                LOG_ERR("Error filling tx fifo from irq! %d", nbytes);
                return;
            }

            tx->off += nbytes;
        }

        if (uart_irq_tx_complete(dev) && (tx->len == tx->off)) {
            atomic_set(&tx->in_progress, 0);
            uart_irq_tx_disable(dev);
            k_sem_give(&tx_done);
        }
    }

    // RX side
    line_data_t *const rx = &data->rx;
    while (uart_irq_rx_ready(dev)) {
        uint8_t byte = 0;
        const int nbytes = uart_fifo_read(dev, &byte, 1);
        if (nbytes < 0) {
            LOG_ERR("Error reading from rx fifo in irq! %d", nbytes);
            return;
        }

        if (nbytes == 0) {
            continue;
        }

        const int message_complete = uart_handle_rxed_byte(rx, &data->rx_cobs_state, byte);
        if (message_complete < 0) {
            LOG_WRN("Error parsing cobs message, dropping current progress");
        } else if (message_complete == 1) {
            data->rx_callback(rx->ptr, rx->off);
            rx->off = 0;
        }
    }
}

static int encode_as_cobs_data(uint8_t *const cobs_buf, const int cobs_buf_len,
                               const struct zmk_inter_kb_msg *msg) {
    const uint8_t *src_data = (const uint8_t *)msg;
    int src_len = sizeof(enum zmk_kb_msg_type) + sizeof(uint16_t);

    switch (msg->type) {
    case ZmkKbMsgTypeBehavior:
        src_len += sizeof(struct zmk_behavior_split_msg);
        break;
    case ZmkKbMsgTypePositionState:
        src_len += sizeof(struct zmk_position_state_msg);
        break;
    default:
        LOG_WRN("Unknown message type: %d", msg->type);
        return -EINVAL;
    }

    int last_zero_offset = 0;
    int offset = 1;

    for (int i = 0; i < src_len; i++) {
        if (offset >= cobs_buf_len) {
            LOG_WRN("Out of memory while encoding source message with size %d", src_len);
            return -ENOMEM;
        }

        if (src_data[i] == 0) {
            cobs_buf[last_zero_offset] = offset - last_zero_offset;
            last_zero_offset = offset++;
        } else {
            cobs_buf[offset++] = src_data[i];
        }
    }

    if (offset >= cobs_buf_len) {
        LOG_WRN("Out of memory while encoding source message with size %d", src_len);
        return -ENOMEM;
    }
    cobs_buf[last_zero_offset] = offset - last_zero_offset;
    cobs_buf[offset++] = 0; // Marks end of frame

    return offset;
}

int zmk_split_serial_send(const struct zmk_inter_kb_msg *msg) {
    if (!uart_data.ready) {
        LOG_WRN("Unable to send data! Uart not ready");
        return -EAGAIN;
    }

    __ASSERT(atomic_get(&uart_data.tx.in_progress) == 0,
             "A TX operation is already in progress! This can only happen if "
             "split_serial_sync_send was called from another thread.");

    // TODO(javier-varez): Calculate CRC and fix it
    static uint8_t cobs_message[MAX_COBS_MSG_LEN];

    int encoded_length = encode_as_cobs_data(cobs_message, MAX_COBS_MSG_LEN, msg);
    if (encoded_length < 0) {
        LOG_ERR("Error encoding cobs message: %d", encoded_length);
        return encoded_length;
    }

    uart_data.tx.ptr = cobs_message;
    uart_data.tx.len = encoded_length;
    uart_data.tx.off = 0;
    atomic_set(&uart_data.tx.in_progress, 1);

    uart_irq_tx_enable(uart_data.serial_dev);

    k_sem_take(&tx_done, K_FOREVER);

    return 0;
}

int zmk_split_serial_sync_init(zmk_split_serial_rx_callback_t rx_callback) {
    if (!device_is_ready(uart_data.serial_dev)) {
        LOG_ERR("UART device:%s not ready", uart_data.serial_dev->name);
        return -EAGAIN;
    }

    static uint8_t rx_buffer[MAX_COBS_MSG_LEN];
    uart_data.rx.ptr = rx_buffer;
    uart_data.rx.len = MAX_COBS_MSG_LEN;
    uart_data.rx.off = 0;
    uart_data.rx_callback = rx_callback;
    uart_data.ready = true;
    uart_irq_callback_user_data_set(uart_data.serial_dev, uart_irq_callback_user_data, &uart_data);
    uart_irq_rx_enable(uart_data.serial_dev);

    LOG_INF("UART device:%s ready", uart_data.serial_dev->name);
    return 0;
}
