#pragma once
#include "co_i2c_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

using I2cSemAwaiter = co_wq::SemReqAwaiter<cortex_lock>;

// I2C 协程管理器：
//  - I2cManager(num) 传入编号，num=0 可用。
//  - acquire(addr7, clock_hz) 首次初始化并配置目标地址与时钟。
//  - transfer(tx/rx) 以 session 入队，完成后通过 awaiter 返回传输长度。
struct I2cManager {
public:
    explicit I2cManager(int num = 0) : i2c_num_(num), handle_(nullptr) { }
    ~I2cManager();
    void release();

    struct AcquireAwaiter {
        I2cManager& self;
        uint8_t     addr7;
        uint32_t    clock_hz;
        i2c_handle* tmp_handle { nullptr };
        bool        result { false };
        alignas(I2cSemAwaiter) unsigned char inner_storage[sizeof(I2cSemAwaiter)];
        I2cSemAwaiter* inner { nullptr };
        AcquireAwaiter(I2cManager& s, uint8_t a7, uint32_t hz) : self(s), addr7(a7), clock_hz(hz) { }
        bool await_ready()
        {
            // 若已持有句柄：直接尝试使用现有句柄的信号量进行快速路径，避免因 get_init 失败导致误返回 false。
            if (self.handle_) {
                inner = new (inner_storage) I2cSemAwaiter(i2c_handle_sem(self.handle_));
                if (inner->await_ready()) {
                    // 持有情况下允许受保护地重配地址/时钟
                    i2c_handle_set_addr(self.handle_, addr7);
                    if (clock_hz)
                        i2c_handle_set_clock(self.handle_, clock_hz);
                    result = true;
                    return true;
                }
                return false;
            }
            tmp_handle = i2c_handle_get_init(self.i2c_num_);
            if (!tmp_handle) {
                result = false;
                return true;
            }
            inner = new (inner_storage) I2cSemAwaiter(i2c_handle_sem(tmp_handle));
            if (inner->await_ready()) {
                self.handle_ = tmp_handle;
                i2c_handle_set_addr(self.handle_, addr7);
                if (clock_hz)
                    i2c_handle_set_clock(self.handle_, clock_hz);
                result = true;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        bool await_resume()
        {
            if (!self.handle_ && tmp_handle) {
                self.handle_ = tmp_handle;
            }
            if (self.handle_) {
                i2c_handle_set_addr(self.handle_, addr7);
                if (clock_hz)
                    i2c_handle_set_clock(self.handle_, clock_hz);
            }
            result = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await(uint8_t addr7, uint32_t clock_hz) { return AcquireAwaiter(*this, addr7, clock_hz); }
    static_assert(std::is_trivially_destructible_v<I2cSemAwaiter>, "SemReqAwaiter must remain trivially destructible");

    struct TransferAwaiter {
        I2cManager& self;
        uint8_t*    buf;
        size_t      len;
        uint32_t    ctrl_bit; // i2c_start_i2c | i2c_end_i2c | i2c_tx_not_rx
        int         result_len { 0 };
        struct co_i2c_session : i2c_session {
            explicit co_i2c_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };
        co_i2c_session node;
        alignas(I2cSemAwaiter) unsigned char inner_storage[sizeof(I2cSemAwaiter)];
        I2cSemAwaiter* inner { nullptr };
        TransferAwaiter(I2cManager& s, uint8_t* b, size_t l, uint32_t cb)
            : self(s), buf(b), len(l), ctrl_bit(cb), node(i2c_handle_wq(s.handle_))
        {
        }
        bool await_ready()
        {
            if (!self.handle_) {
                result_len = 0;
                return true;
            }
            node.buf       = buf;
            node.trans_max = static_cast<uint32_t>(len);
            node.trans_len = 0;
            node.len       = 0;
            node.ctrl_bit  = ctrl_bit;
            node.func      = [](co_wq::worknode* pws) {
                auto* psess = static_cast<co_i2c_session*>(pws);
                psess->cpl_inotify.release();
            };
            i2c_enqueue_session(*self.handle_, node);
            inner = new (inner_storage) I2cSemAwaiter(node.cpl_inotify);
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
    TransferAwaiter transfer_await(uint8_t* buf, size_t len, uint32_t ctrl_bit)
    {
        return TransferAwaiter(*this, buf, len, ctrl_bit);
    }

    // 多段传输（模板）：N 为段数，默认 2。传参使用 C 数组引用，调用简洁且零开销。
    template <int N = 2> struct MultiTransferAwaiter {
        I2cManager& self;
        uint8_t* (&bufs)[N];
        size_t (&lens)[N];
        uint32_t (&ctrls)[N];
        int result_len { 0 }; // 返回最后一段的完成长度
        struct co_i2c_session : i2c_session {
            explicit co_i2c_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };
        co_i2c_session nodes[N] = { co_i2c_session(i2c_handle_wq(self.handle_)) };
        alignas(I2cSemAwaiter) unsigned char inner_storage[sizeof(I2cSemAwaiter)];
        I2cSemAwaiter* inner { nullptr };
        MultiTransferAwaiter(I2cManager& s, uint8_t* (&b)[N], size_t (&l)[N], uint32_t (&c)[N])
            : self(s), bufs(b), lens(l), ctrls(c)
        {
            // nodes[0] 已构造，剩余用就地构造
            for (int i = 1; i < N; ++i) {
                new (&nodes[i]) co_i2c_session(i2c_handle_wq(self.handle_));
            }
        }
        bool await_ready()
        {
            if (!self.handle_) {
                result_len = 0;
                return true;
            }
            // 准备所有段，最后一段安装回调以唤醒
            for (int i = 0; i < N; ++i) {
                auto& node     = nodes[i];
                node.buf       = bufs[i];
                node.trans_max = static_cast<uint32_t>(lens[i]);
                node.trans_len = 0;
                node.len       = 0;
                node.ctrl_bit  = ctrls[i];
                node.func      = nullptr;
            }
            nodes[N - 1].func = [](co_wq::worknode* pws) {
                auto* psess = static_cast<co_i2c_session*>(pws);
                psess->cpl_inotify.release();
            };

            // 逐段入队，启动传输
            for (int i = 0; i < N; ++i) {
                i2c_enqueue_session(*self.handle_, nodes[i]);
            }

            inner = new (inner_storage) I2cSemAwaiter(nodes[N - 1].cpl_inotify);
            if (inner->await_ready()) {
                result_len = static_cast<int>(nodes[N - 1].len);
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = static_cast<int>(nodes[N - 1].len);
            return result_len;
        }
    };

    template <int N = 2>
    MultiTransferAwaiter<N> transfer_multi_await(uint8_t* (&bufs)[N], size_t (&lens)[N], uint32_t (&ctrl_bits)[N])
    {
        return MultiTransferAwaiter<N>(*this, bufs, lens, ctrl_bits);
    }

private:
    int         i2c_num_ { 0 };
    i2c_handle* handle_;
};
