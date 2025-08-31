#include "../internal/spi_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"
#include <cstdint>

typedef enum {
    spi_is_master_not_slave = 1 << 0,
    spi_is_8bit_not_16bit   = 1 << 1,
    spi_is_msb_not_lsb      = 1 << 2,

    spi_cpol = 1 << 3,
    spi_cpha = 1 << 4,

    spi_cp_mode_0 = 0,
    spi_cp_mode_1 = (0 | spi_cpha),
    spi_cp_mode_2 = (spi_cpol | 0),
    spi_cp_mode_3 = (spi_cpol | spi_cpha),

    spi_high_speed = 1 << 5,
} spi_mode_bit;

typedef enum {
    spi_cs_not_set_at_end  = 1 << 0,
    spi_cs_not_set_at_half = 1 << 1,
    spi_only_together      = 1 << 2,
    spi_short_cb           = 1 << 3,
    spi_fast_cb            = 1 << 4,
} spi_ctrl_bit;

struct spi_handle;
struct SpiManager {
public:
    SpiManager() : handle_(nullptr) { }
    ~SpiManager();
    // 可自定义协程帧分配器 Alloc（默认 sys_taskalloc）
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> init(uint32_t mode);
    // SPI 全双工/半双工传输，tx_buff 或 rx_buff 可为 nullptr
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
    transfer(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit);

private:
    spi_handle* handle_;
};

// ============== 模板实现 ==============

// 初始化 SPI（仅第一次真正配置硬件）；若已初始化直接返回 true。
template <typename Alloc>
co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> SpiManager::init(uint32_t mode)
{
    if (handle_) {
        co_return true; // 已初始化
    }
    auto tmp = spi_handle_get_init();
    if (!tmp) {
        co_return false; // 获取失败
    }
    co_await co_wq::SemReqAwaiter(spi_handle_sem(tmp)); // 进入互斥
    handle_ = tmp;
    if (mode != spi_handle_get_mode(handle_)) {
        spi_handle_config_mode(handle_, mode); // 重新配置模式
    }
    spi_ext_set_sp_dummy_byte(handle_, 0xff); // 设定 dummy 字节
    co_return true;
}

// SPI 传输：根据 ctrl_bit 控制片选/回调行为（保持原逻辑）。
template <typename Alloc>
co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
SpiManager::transfer(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit)
{
    struct co_spi_session : spi_session {
        explicit co_spi_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1) { INIT_LIST_HEAD(&ws_node); }
        co_wq::Semaphore<cortex_lock> cpl_inotify; // 传输完成信号
    };

    co_spi_session node(spi_handle_wq(handle_));
    node.tx_buf   = tx_buff;
    node.rx_buf   = rx_buff;
    node.len      = len;
    node.ctrl_bit = ctrl_bit;

    node.func = [](co_wq::worknode* pws) {
        auto* psess = static_cast<co_spi_session*>(pws);
        psess->cpl_inotify.release(); // 唤醒等待协程
    };

    spi_transfer_setup(*handle_, node); // 放入队列 / 可能立即启动
    co_await co_wq::SemReqAwaiter(node.cpl_inotify);
    co_return node.len; // 返回已传输长度
}
