// UART 内部结构与辅助访问函数声明
// 仅供协程模板 / 驱动实现使用，不对外暴露为公共 API。
#pragma once

#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

// 标志位：区分收发 / 超时 / 快速回调路径
struct UART_FLG {
    uint32_t tx_not_rx  : 1; // 1: 发送(TX)  0: 接收(RX)
    uint32_t rx_timeout : 8; // 接收超时阈值 (0 表示不使能)
    uint32_t fast_cb    : 1; // 是否走快速回调路径
};

// 会话节点：继承 worknode 用于放入工作队列
struct uart_session : co_wq::worknode {
    uint8_t*        buff;    // 数据缓冲区（收/发共用）
    uint32_t        len;     // 期望长度
    uint32_t        cur_len; // 已完成长度
    struct UART_FLG flg;     // 标志
};

struct uart_handle; // 前向声明（具体定义在 co_uart.cpp）

// 句柄获取（按编号）
struct uart_handle* uart_handle_get(int num);
// 访问底层互斥信号量（用于 init 协程获取独占）
co_wq::Semaphore<cortex_lock>& uart_handle_get(struct uart_handle* puart);
// 提交一次传输/接收会话（内部加锁，可能立即启动硬件）
int uart_ext_transfer_cb(struct uart_handle* handle, struct uart_session& psess);
// 获取句柄绑定的工作队列
co_wq::workqueue<cortex_lock>& uart_handle_wq(struct uart_handle* handle);
