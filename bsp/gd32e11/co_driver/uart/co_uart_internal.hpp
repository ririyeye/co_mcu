// Internal UART definitions to allow templated coroutine methods with custom allocators.
// This header is intentionally kept minimal and not installed as a public API.
#pragma once

#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

struct UART_FLG {
    uint32_t tx_not_rx  : 1; // 1: TX, 0: RX
    uint32_t rx_timeout : 8; // timeout threshold (0 means disabled)
    uint32_t fast_cb    : 1; // fast callback path
};

struct uart_session : co_wq::worknode {
    uint8_t*        buff;
    uint32_t        len;
    uint32_t        cur_len; // current transferred length
    struct UART_FLG flg;
};

struct uart_handle; // fwd

// Forward declarations for functions implemented in co_uart.cpp
struct uart_handle*            uart_handle_get(int num);
co_wq::Semaphore<cortex_lock>& uart_handle_get(struct uart_handle* puart);
int                            uart_ext_transfer_cb(struct uart_handle* handle, struct uart_session& psess);
co_wq::workqueue<cortex_lock>& uart_handle_wq(struct uart_handle* handle);
// 动态重新配置波特率（需要已 acquire 并持有互斥）
int uart_handle_set_baudrate(struct uart_handle* handle, int baud);
