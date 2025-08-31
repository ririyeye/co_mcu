// UART 协程管理器接口
// 提供 init 与 uart_transfer 两个协程接口，支持自定义协程帧分配器 Alloc
// 使用 intrusive workqueue + 中断驱动完成异步收发。
#include "../internal/uart_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"
#include <cstdint>

struct uart_handle;
struct UartManager {
public:
    explicit UartManager(int uart_num) : uart_num_(uart_num), handle_(nullptr) { }
    ~UartManager();
    // 初始化指定编号 UART，首次调用会真正配置硬件；再次调用直接返回 true。
    // 支持自定义协程帧分配器 Alloc（默认使用 sys_taskalloc）
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> init();
    // 异步收发：
    //   tx=1 发送 data[len]；tx=0 接收 data[len]（阻塞直到接满或超时触发完成）
    // 返回实际完成字节数。
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc> uart_transfer(uint8_t* data, size_t len, int tx);

private:
    int          uart_num_;
    uart_handle* handle_; // 获取后保持，析构时释放其互斥信号量
};

// ---- template implementations ----

template <typename Alloc> co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> UartManager::init()
{
    if (handle_) {
        co_return true; // 已初始化
    }
    auto tmp_handle = uart_handle_get(uart_num_);
    if (!tmp_handle) {
        co_return false; // 获取失败（编号无效）
    }
    // 获取互斥，保证只有一个协程完成真正硬件初始化
    co_await co_wq::SemReqAwaiter(uart_handle_get(tmp_handle));
    handle_ = tmp_handle;
    co_return true;
}

template <typename Alloc>
co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
UartManager::uart_transfer(uint8_t* data, size_t len, int tx)
{
    // 局部派生类型：附加一个完成信号量对象
    struct tx_uart_session : uart_session {
        explicit tx_uart_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
        {
            INIT_LIST_HEAD(&ws_node);
        }
        co_wq::Semaphore<cortex_lock> cpl_inotify; // 完成通知
    };

    tx_uart_session node(uart_handle_wq(handle_));
    node.buff          = const_cast<uint8_t*>(data);
    node.len           = len;
    node.cur_len       = 0;
    node.flg.tx_not_rx = !!tx; // tx!=0 表示发送

    node.func = [](co_wq::worknode* pws) {
        auto* psess = static_cast<tx_uart_session*>(pws);
        psess->cpl_inotify.release(); // 回调中唤醒等待协程
    };

    uart_ext_transfer_cb(handle_, node);             // 提交到硬件队列（内部加锁）
    co_await co_wq::SemReqAwaiter(node.cpl_inotify); // 等待完成
    co_return node.cur_len;                          // 返回实际传输字节数
}
