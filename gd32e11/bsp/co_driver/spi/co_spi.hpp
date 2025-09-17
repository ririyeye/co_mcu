#pragma once
#include "co_spi_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

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
// SPI 协程管理器：支持编号构造（目前仅实现 spi0）。
//  - SpiManager(num) 传入编号，num=0 可用，其它返回 acquire=false。
//  - acquire(mode) 首次初始化并根据 mode 配置（后续若 mode 不同将自动重新配置）。
//  - transfer() 支持全双工，tx/rx 缓冲可独立为 nullptr，内部使用 DMA + 工作队列。
struct SpiManager {
public:
    explicit SpiManager(int num = 0) : spi_num_(num), handle_(nullptr) { }
    ~SpiManager();
    void release(); // 主动释放（等价析构内部逻辑）
    // 访问底层原始 spi_handle（只读/高级配置用）
    spi_handle* raw_handle() const { return handle_; }
    // 设置 dummy byte（在部分 Flash 等设备读操作需要特定 dummy）
    void set_dummy(uint8_t b)
    {
        if (handle_) {
            spi_ext_set_sp_dummy_byte(handle_, b);
        }
    }
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
            tmp_handle = spi_handle_get_init(self.spi_num_);
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
            // 仅当当前尚未持有句柄且本次确实获取到句柄时，才进行赋值与配置。
            if (!self.handle_ && tmp_handle) {
                self.handle_ = tmp_handle;
                if (mode != spi_handle_get_mode(self.handle_)) {
                    spi_handle_config_mode(self.handle_, mode);
                }
                spi_ext_set_sp_dummy_byte(self.handle_, 0xff);
            }
            result = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await(uint32_t mode) { return AcquireAwaiter(*this, mode); }
    static_assert(std::is_trivially_destructible_v<SemAwaiter>, "SemReqAwaiter must remain trivially destructible");

    // 统一的传输 Awaiter（模板）：N 段，N=1 时等价单次传输。
    // 规则：若 ctrl 含有 spi_cs_not_set_at_half，则对所有非最后一段额外 OR 上 spi_cs_not_set_at_end，
    // 从而保证多段期间 CS 保持不抬高，直到最后一段按 ctrl 处理。
    template <int N = 2> struct TransferAwaiter {
        SpiManager& self;
        // 内部持有一份拷贝，避免包装函数中局部数组生命周期问题
        const uint8_t* txs[N];
        const uint8_t* rxs[N];
        size_t         lens[N];
        uint32_t       ctrl;             // 单一 ctrl_bit，内部按段规则分配
        int            result_len { 0 }; // 返回最后一段完成长度
        struct co_spi_session : spi_session {
            explicit co_spi_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };
        alignas(co_spi_session) unsigned char nodes_storage[sizeof(co_spi_session) * N];
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };
        TransferAwaiter(SpiManager& s, const uint8_t* (&t)[N], const uint8_t* (&r)[N], size_t (&l)[N], uint32_t c)
            : self(s), ctrl(c)
        {
            for (int i = 0; i < N; ++i) {
                txs[i]  = t[i];
                rxs[i]  = r[i];
                lens[i] = l[i];
                new (&nodes_ptr()[i]) co_spi_session(spi_handle_wq(self.handle_));
            }
        }
        bool await_ready()
        {
            if (!self.handle_) {
                result_len = 0;
                return true;
            }
            const bool use_at_half = (ctrl & spi_cs_not_set_at_half) != 0;
            for (int i = 0; i < N; ++i) {
                auto& node    = nodes_ptr()[i];
                node.tx_buf   = txs[i];
                node.rx_buf   = rxs[i];
                node.len      = lens[i];
                node.ctrl_bit = ctrl;
                if (use_at_half && i < (N - 1)) {
                    node.ctrl_bit |= spi_cs_not_set_at_end; // 非最后一段强制不在段末置 CS
                }
                node.func = nullptr;
            }
            // 最后一段负责唤醒
            nodes_ptr()[N - 1].func = [](co_wq::worknode* pws) {
                auto* psess = static_cast<co_spi_session*>(pws);
                psess->cpl_inotify.release();
            };

            // 入队所有段
            for (int i = 0; i < N; ++i) {
                spi_enqueue_session(*self.handle_, nodes_ptr()[i]);
            }

            inner = new (inner_storage) SemAwaiter(nodes_ptr()[N - 1].cpl_inotify);
            if (inner->await_ready()) {
                result_len = static_cast<int>(nodes_ptr()[N - 1].len);
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = static_cast<int>(nodes_ptr()[N - 1].len);
            return result_len;
        }

    private:
        inline co_spi_session* nodes_ptr() { return reinterpret_cast<co_spi_session*>(nodes_storage); }
    };

    // 单段传输便捷接口：通过模板 N=1 复用统一实现。
    TransferAwaiter<1> transfer_await(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit)
    {
        const uint8_t* txs[1]  = { tx_buff };
        const uint8_t* rxs[1]  = { rx_buff };
        size_t         lens[1] = { len };
        return TransferAwaiter<1>(*this, txs, rxs, lens, ctrl_bit);
    }

    // 多段传输接口：返回统一的模板 Awaiter
    template <int N = 2>
    TransferAwaiter<N>
    transfer_multi_await(const uint8_t* (&txs)[N], const uint8_t* (&rxs)[N], size_t (&lens)[N], uint32_t ctrl_bit)
    {
        return TransferAwaiter<N>(*this, txs, rxs, lens, ctrl_bit);
    }

private:
    int         spi_num_ { 0 };
    spi_handle* handle_;
};
