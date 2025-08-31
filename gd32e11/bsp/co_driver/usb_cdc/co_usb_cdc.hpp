#include "../internal/usb_cdc_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"
#include <cstdint>

struct cdc_usr;
struct UsbCDCManager {
public:
    UsbCDCManager() : handle_(nullptr) { }
    ~UsbCDCManager();
    // 可自定义协程帧分配器 Alloc
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> init();
    // tx=1 发送; tx=0 接收（阻塞等待满或数据到来即完成）
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc> transfer(uint8_t* data, size_t len, int tx);

private:
    cdc_usr* handle_;
};

// ============== 模板实现 ==============

// 初始化 USB CDC 设备（只初始化一次）
template <typename Alloc> co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> UsbCDCManager::init()
{
    if (handle_) {
        co_return true;
    }
    auto tmp = get_cdc_init();
    if (!tmp) {
        co_return false;
    }
    co_await co_wq::SemReqAwaiter(cdc_usr_sem(tmp));
    handle_ = tmp;
    co_return true;
}

// USB CDC 传输/接收
template <typename Alloc>
co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
UsbCDCManager::transfer(uint8_t* data, size_t len, int tx)
{
    struct cdc_usr_node_cb : cdc_usr_node {
        explicit cdc_usr_node_cb(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1) { }
        co_wq::Semaphore<cortex_lock> cpl_inotify; // 完成通知
    };

    cdc_usr_node_cb node(cdc_usr_wq(handle_));
    node.data    = data;
    node.dat_max = len;
    node.dat_cur = 0;

    node.func = [](co_wq::worknode* pws) {
        auto* psess = static_cast<cdc_usr_node_cb*>(pws);
        psess->cpl_inotify.release();
    };

    uint32_t lk = lock_acquire();
    cdc_usr_async_setup_nolock(tx ? cdc_usr_list_tx(handle_) : cdc_usr_list_rx(handle_),
                               &node,
                               cdc_usr_ready(handle_),
                               cdc_usr_dev(handle_),
                               tx);
    lock_release(lk);

    co_await co_wq::SemReqAwaiter(node.cpl_inotify);
    co_return node.dat_cur;
}
