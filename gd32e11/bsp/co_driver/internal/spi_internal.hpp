// SPI 内部结构与辅助函数声明
// 仅供协程模板 / 驱动实现使用，不作为公共 API 暴露。
#pragma once

#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

struct spi_handle; // 前向声明（定义在 co_spi.cpp）

// SPI 传输会话节点（放入等待链表，由中断/DMA 驱动）
struct spi_session : co_wq::worknode {
    const uint8_t* tx_buf;   // 发送缓冲（可为 nullptr 表示仅接收）
    const uint8_t* rx_buf;   // 接收缓冲（可为 nullptr 表示仅发送）
    uint32_t       len;      // 传输长度
    uint32_t       ctrl_bit; // 控制位（片选/回调策略等）
};

// 获取/初始化 SPI 硬件句柄（若已初始化则直接返回）
struct spi_handle* spi_handle_get_init();
// 访问底层工作队列
co_wq::workqueue<cortex_lock>& spi_handle_wq(struct spi_handle* h);
// 访问句柄内部信号量
co_wq::Semaphore<cortex_lock>& spi_handle_sem(struct spi_handle* h);
// 获取当前模式
uint32_t spi_handle_get_mode(struct spi_handle* h);
// 若模式不同则配置（内部会调用 spi_master_config）
void spi_handle_config_mode(struct spi_handle* h, uint32_t mode);
// 设置 dummy 字节
void spi_ext_set_sp_dummy_byte(struct spi_handle* phandle, uint8_t dummy_byte);
// 提交一次传输（在空队列时可能立即启动）
void spi_transfer_setup(struct spi_handle& phandle, struct spi_session& psess);
