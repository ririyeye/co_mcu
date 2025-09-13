// UART 协程管理器接口 (支持编号获取：uart0 / uart1)
//  - 构造 UartManager(num) 指定 UART 号，目前实现 0 与 1。
//  - acquire() 仅首次进行硬件初始化，后续复用。
//  - set_baudrate()
//  可在持有互斥下动态重设波特率（简单复位串口寄存器，不影响队列中的会话指针，但调用时应确保无进行中的会话以避免竞态）。
//  - uart_transfer(): tx=1 发送; tx=0 接收；接收支持 rx_timeout / fast_cb 预留位（当前 fast_cb 未实现回调快速路径）。
// 使用 intrusive workqueue + 中断驱动完成异步收发。
#include "co_uart_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

using SemAwaiter = co_wq::SemReqAwaiter<cortex_lock>;

struct uart_handle;
struct UartManager {
public:
    explicit UartManager(int uart_num) : uart_num_(uart_num), handle_(nullptr) { }
    ~UartManager();
    void release();
    // acquire 指定编号 UART（首次真正配置），再次调用直接返回 true。
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> acquire();
    // 轻量 awaiter（包装内部 SemReqAwaiter，复用其逻辑，避免重复代码）
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
            self.handle_ = tmp_handle; // 信号量获取成功
            result       = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await() { return AcquireAwaiter(*this); }
    // 防御：如果以后 SemReqAwaiter 变为非平凡析构，需要同步调整 InitAwaiter 析构。
    static_assert(std::is_trivially_destructible_v<SemAwaiter>, "SemReqAwaiter must remain trivially destructible");
    // 异步收发：
    //   tx=1 发送 data[len]；tx=0 接收 data[len]（阻塞直到接满或超时触发完成）
    // 返回实际完成字节数。
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc> uart_transfer(uint8_t* data, size_t len, int tx);
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
    // 轻量收发 awaiter（与原实现等价，包装内部 session + SemReqAwaiter 逻辑）
    struct TransferAwaiter {
        UartManager& self;
        uint8_t*     data;
        size_t       len;
        int          tx;
        int          result_len { 0 };

        struct tx_uart_session : uart_session {
            explicit tx_uart_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify; // 完成通知
        };

        // 延迟构造 session 需要 wq，直接在构造函数里用 self.handle_（假定已 init）。
        tx_uart_session node;

        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };

        TransferAwaiter(UartManager& u, uint8_t* d, size_t l, int t)
            : self(u), data(d), len(l), tx(t), node(uart_handle_wq(u.handle_))
        {
        }

        bool await_ready()
        {
            if (!self.handle_) { // 未初始化：与原实现不同，这里直接返回 0；假定调用方应先 init。
                result_len = 0;
                return true;
            }
            node.buff          = data;
            node.len           = len;
            node.cur_len       = 0;
            node.flg.tx_not_rx = !!tx;
            node.func          = [](co_wq::worknode* pws) {
                auto* psess = static_cast<tx_uart_session*>(pws);
                psess->cpl_inotify.release();
            };
            uart_ext_transfer_cb(self.handle_, node); // 提交
            inner = new (inner_storage) SemAwaiter(node.cpl_inotify);
            if (inner->await_ready()) { // 立即完成
                result_len = node.cur_len;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = node.cur_len;
            return result_len;
        }
    };
    TransferAwaiter transfer_await(uint8_t* data, size_t len, int tx) { return TransferAwaiter(*this, data, len, tx); }

private:
    int          uart_num_;
    uart_handle* handle_; // 获取后保持，析构时释放其互斥信号量
};

// ---- template implementations ----

template <typename Alloc> co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> UartManager::acquire()
{
    co_return co_await acquire_await();
}

template <typename Alloc>
co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
UartManager::uart_transfer(uint8_t* data, size_t len, int tx)
{
    co_return co_await transfer_await(data, len, tx);
}
