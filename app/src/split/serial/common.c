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

const struct device *const serial_dev = DEVICE_DT_GET(UART_NODE1);
static int uart_ready = 0;

K_SEM_DEFINE(tx_done, 0, 1);
K_SEM_DEFINE(rx_done, 0, 1);

typedef struct uart_data {
    const uint8_t *tx_ptr;
    int tx_len;
    atomic_t tx_in_progress;
    uint8_t *rx_ptr;
    int rx_len;
    atomic_t rx_in_progress;
} uart_data_t;

static uart_data_t uart_data = {};

static void uart_irq_callback_user_data(const struct device *const dev, void *const user_data) {
    uart_data_t *const data = (uart_data_t *)user_data;
    if (!uart_irq_update(dev)) {
        return;
    }

    if (atomic_get(&data->tx_in_progress) != 0) {
        while ((data->tx_len > 0) && uart_irq_tx_ready(dev)) {
            const int nbytes = uart_fifo_fill(dev, &data->tx_ptr[0], data->tx_len);
            if (nbytes < 0) {
                LOG_ERR("Error filling tx fifo from irq! %d", nbytes);
                return;
            }

            data->tx_ptr += nbytes;
            data->tx_len -= nbytes;
        }

        if (uart_irq_tx_complete(dev) && (data->tx_len == 0)) {
            atomic_set(&data->tx_in_progress, 0);
            uart_irq_tx_disable(dev);
            k_sem_give(&tx_done);
        }
    }

    if (atomic_get(&data->rx_in_progress) != 0) {
        const bool rx_was_not_zero = data->rx_len > 0;
        while ((data->rx_len > 0) && uart_irq_rx_ready(dev)) {
            const int nbytes = uart_fifo_read(dev, &data->rx_ptr[0], data->rx_len);
            if (nbytes < 0) {
                LOG_ERR("Error reading from rx fifo in irq! %d", nbytes);
                return;
            }

            data->rx_ptr += nbytes;
            data->rx_len -= nbytes;
        }

        if (rx_was_not_zero && (data->rx_len == 0)) {
            uart_irq_rx_disable(dev);
            atomic_set(&data->rx_in_progress, 0);
            k_sem_give(&rx_done);
        }
    }
}

void split_serial_sync_send(const uint8_t *const data, const size_t len) {
    if (!uart_ready) {
        LOG_WRN("Unable to send data! Uart not ready");
        return;
    }

    uart_data.tx_ptr = data;
    uart_data.tx_len = len;
    atomic_set(&uart_data.tx_in_progress, 1);
    uart_irq_tx_enable(serial_dev);

    k_sem_take(&tx_done, K_FOREVER);
}

void split_serial_sync_recv(uint8_t *const data, const size_t len) {
    if (!uart_ready) {
        LOG_WRN("Unable to recv data! Uart not ready");
        return;
    }

    uart_data.rx_ptr = data;
    uart_data.rx_len = len;
    atomic_set(&uart_data.rx_in_progress, 1);

    uart_irq_rx_enable(serial_dev);
    k_sem_take(&rx_done, K_FOREVER);
}

void split_serial_sync_init() {
    if (!device_is_ready(serial_dev)) {
        LOG_ERR("UART device:%s not ready", serial_dev->name);
        return;
    }

    uart_irq_callback_user_data_set(serial_dev, uart_irq_callback_user_data, &uart_data);

    uart_ready = 1;
    LOG_INF("UART device:%s ready", serial_dev->name);
}
