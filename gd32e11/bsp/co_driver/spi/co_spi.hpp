#include "co_spi_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

using SemAwaiter = co_wq::SemReqAwaiter<cortex_lock>;

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
    void release(); // 主动释放（等价析构内部逻辑）
    // 可自定义协程帧分配器 Alloc（默认 sys_taskalloc）
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> acquire(uint32_t mode);
    // SPI 全双工/半双工传输，tx_buff 或 rx_buff 可为 nullptr
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
    transfer(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit);

    // ---- Awaiters ----
    struct AcquireAwaiter {
        SpiManager& self;
        uint32_t    mode;
        spi_handle* tmp_handle { nullptr };
        bool        result { false };
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };
        AcquireAwaiter(SpiManager& s, uint32_t m) : self(s), mode(m) { }
        bool await_ready()
        {
            if (self.handle_) { // 已初始化（保持原逻辑：不重新配置）
                result = true;
                return true;
            }
            tmp_handle = spi_handle_get_init();
            if (!tmp_handle) {
                result = false;
                return true;
            }
            inner = new (inner_storage) SemAwaiter(spi_handle_sem(tmp_handle));
            if (inner->await_ready()) { // 立即进入
                self.handle_ = tmp_handle;
                if (mode != spi_handle_get_mode(self.handle_)) {
                    spi_handle_config_mode(self.handle_, mode);
                }
                spi_ext_set_sp_dummy_byte(self.handle_, 0xff);
                result = true;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        bool await_resume()
        {
            self.handle_ = tmp_handle;
            if (self.handle_ && mode != spi_handle_get_mode(self.handle_)) {
                spi_handle_config_mode(self.handle_, mode);
            }
            if (self.handle_) {
                spi_ext_set_sp_dummy_byte(self.handle_, 0xff);
            }
            result = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await(uint32_t mode) { return AcquireAwaiter(*this, mode); }
    static_assert(std::is_trivially_destructible_v<SemAwaiter>, "SemReqAwaiter must remain trivially destructible");

    struct TransferAwaiter {
        SpiManager&    self;
        const uint8_t* tx_buff;
        const uint8_t* rx_buff;
        size_t         len;
        uint32_t       ctrl_bit;
        int            result_len { 0 };
        struct co_spi_session : spi_session {
            explicit co_spi_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };
        co_spi_session node;
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };
        TransferAwaiter(SpiManager& s, const uint8_t* tx, const uint8_t* rx, size_t l, uint32_t cb)
            : self(s), tx_buff(tx), rx_buff(rx), len(l), ctrl_bit(cb), node(spi_handle_wq(s.handle_))
        {
        }
        bool await_ready()
        {
            if (!self.handle_) {
                result_len = 0;
                return true;
            }
            node.tx_buf   = tx_buff;
            node.rx_buf   = rx_buff;
            node.len      = len;
            node.ctrl_bit = ctrl_bit;
            node.func     = [](co_wq::worknode* pws) {
                auto* psess = static_cast<co_spi_session*>(pws);
                psess->cpl_inotify.release();
            };
            spi_transfer_setup(*self.handle_, node);
            inner = new (inner_storage) SemAwaiter(node.cpl_inotify);
            if (inner->await_ready()) {
                result_len = static_cast<int>(node.len);
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = static_cast<int>(node.len);
            return result_len;
        }
    };
    TransferAwaiter transfer_await(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit)
    {
        return TransferAwaiter(*this, tx_buff, rx_buff, len, ctrl_bit);
    }

private:
    spi_handle* handle_;
};

// ============== 模板实现 ==============

// acquire SPI（仅第一次真正配置硬件）；若已获取则直接返回 true。
template <typename Alloc>
co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> SpiManager::acquire(uint32_t mode)
{
    co_return co_await acquire_await(mode);
}

// SPI 传输：根据 ctrl_bit 控制片选/回调行为（保持原逻辑）。
template <typename Alloc>
co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
SpiManager::transfer(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit)
{
    co_return co_await transfer_await(tx_buff, rx_buff, len, ctrl_bit);
}
