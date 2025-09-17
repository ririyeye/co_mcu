// UART 协程管理器接口 (支持编号获取：uart0 / uart1)
//  - 构造 UartManager(num) 指定 UART 号，目前实现 0 与 1。
//  - acquire() 仅首次进行硬件初始化，后续复用。
//  - set_baudrate()
//  可在持有互斥下动态重设波特率（简单复位串口寄存器，不影响队列中的会话指针，但调用时应确保无进行中的会话以避免竞态）。
//  - uart_transfer(): tx=1 发送; tx=0 接收；接收支持 rx_timeout / fast_cb 预留位（当前 fast_cb 未实现回调快速路径）。
// 使用 intrusive workqueue + 中断驱动完成异步收发。
#pragma once
#include "co_uart_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

using SemAwaiter = co_wq::SemReqAwaiter<cortex_lock>;

struct uart_handle;
struct UartManager {
public:
    explicit UartManager(int uart_num) : uart_num_(uart_num), handle_(nullptr) { }
    ~UartManager();
    void release();

    struct AcquireAwaiter {
        UartManager& self;
        uart_handle* tmp_handle { nullptr };
        bool         result { false };
        // 就地存储内部的 SemReqAwaiter（避免动态分配）
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };

        explicit AcquireAwaiter(UartManager& u) : self(u) { }

        bool await_ready()
        {
            if (self.handle_) { // 已初始化
                result = true;
                return true;
            }
            tmp_handle = uart_handle_get(self.uart_num_);
            if (!tmp_handle) { // 获取失败
                result = false;
                return true;
            }
            auto& sem = uart_handle_get(tmp_handle);
            inner     = new (inner_storage) SemAwaiter(sem);
            if (inner->await_ready()) { // 立即获取成功
                self.handle_ = tmp_handle;
                result       = true;
                return true;
            }
            return false; // 需要挂起
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        bool await_resume()
        {
            // 仅在尚未持有句柄时进行赋值，避免并发情况下无意义覆盖。
            if (!self.handle_ && tmp_handle) {
                self.handle_ = tmp_handle; // 信号量获取成功
            }
            result = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await() { return AcquireAwaiter(*this); }
    // 防御：如果以后 SemReqAwaiter 变为非平凡析构，需要同步调整 InitAwaiter 析构。
    static_assert(std::is_trivially_destructible_v<SemAwaiter>, "SemReqAwaiter must remain trivially destructible");
    // 变更波特率（需已 acquire 成功，内部加互斥）。失败返回负数。
    int set_baudrate(int baud)
    {
        if (!handle_)
            return -1;
        auto& sem = ::uart_handle_get(handle_);
        if (!sem.try_acquire()) {
            return -2; // busy
        }
        int ret = uart_handle_set_baudrate(handle_, baud);
        sem.release();
        return ret;
    }
    // 统一模板收发 awaiter：N 段，N=1 等价单次
    template <int N = 2> struct TransferAwaiter {
        UartManager& self;
        // 内部持有数组拷贝，避免包装函数中局部数组悬垂
        uint8_t* datas[N];
        size_t   lens[N];
        int      txs[N];
        int      result_len { 0 };

        struct tx_uart_session : uart_session {
            explicit tx_uart_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify; // 完成通知
        };

        alignas(tx_uart_session) unsigned char nodes_storage[sizeof(tx_uart_session) * N];
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };

        TransferAwaiter(UartManager& u, uint8_t* (&d)[N], size_t (&l)[N], int (&t)[N]) : self(u)
        {
            for (int i = 0; i < N; ++i) {
                datas[i] = d[i];
                lens[i]  = l[i];
                txs[i]   = t[i];
                new (&nodes_ptr()[i]) tx_uart_session(uart_handle_wq(self.handle_));
            }
        }

        bool await_ready()
        {
            if (!self.handle_) { // 未初始化
                result_len = 0;
                return true;
            }
            for (int i = 0; i < N; ++i) {
                auto& node         = nodes_ptr()[i];
                node.buff          = datas[i];
                node.len           = lens[i];
                node.cur_len       = 0;
                node.flg.tx_not_rx = !!txs[i];
                node.func          = nullptr;
            }
            // 最后一段回调唤醒
            nodes_ptr()[N - 1].func = [](co_wq::worknode* pws) {
                auto* psess = static_cast<tx_uart_session*>(pws);
                psess->cpl_inotify.release();
            };
            // 依次提交
            for (int i = 0; i < N; ++i) {
                uart_ext_transfer_cb(self.handle_, nodes_ptr()[i]);
            }
            inner = new (inner_storage) SemAwaiter(nodes_ptr()[N - 1].cpl_inotify);
            if (inner->await_ready()) { // 立即完成
                result_len = nodes_ptr()[N - 1].cur_len;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = nodes_ptr()[N - 1].cur_len;
            return result_len;
        }

    private:
        inline tx_uart_session* nodes_ptr() { return reinterpret_cast<tx_uart_session*>(nodes_storage); }
    };

    // 单段传输便捷接口
    TransferAwaiter<1> transfer_await(uint8_t* data, size_t len, int tx)
    {
        uint8_t* datas[1] = { data };
        size_t   lens[1]  = { len };
        int      txs[1]   = { tx };
        return TransferAwaiter<1>(*this, datas, lens, txs);
    }

    // 多段传输接口
    template <int N = 2> TransferAwaiter<N> transfer_multi_await(uint8_t* (&datas)[N], size_t (&lens)[N], int (&txs)[N])
    {
        return TransferAwaiter<N>(*this, datas, lens, txs);
    }

private:
    int          uart_num_;
    uart_handle* handle_; // 获取后保持，析构时释放其互斥信号量
};
