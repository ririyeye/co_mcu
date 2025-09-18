// SPI 内部结构与辅助函数声明，仅供协程模板实现使用。
#pragma once
#include "semaphore.hpp"
#include "syswork.hpp"

struct spi_handle; // 前向声明
struct spi_session : co_wq::worknode {
    const uint8_t* tx_buf;
    const uint8_t* rx_buf;
    uint32_t       len;
    uint32_t       ctrl_bit;
};

// 按编号获取/初始化 SPI（目前仅支持 0）
struct spi_handle* spi_handle_get_init(int num);
// 兼容旧接口（等价 num=0）
static inline struct spi_handle* spi_handle_get_init()
{
    return spi_handle_get_init(0);
}
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
// 入队一次传输（在空队列时立即启动）
void spi_enqueue_session(struct spi_handle& phandle, struct spi_session& psess);
