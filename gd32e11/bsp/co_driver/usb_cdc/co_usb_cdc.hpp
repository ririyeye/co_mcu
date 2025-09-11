#include "co_usb_cdc_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

using SemAwaiter = co_wq::SemReqAwaiter<cortex_lock>;

struct cdc_usr;
struct UsbCDCManager {
public:
    UsbCDCManager() : handle_(nullptr) { }
    ~UsbCDCManager();
    void release();
    // 可自定义协程帧分配器 Alloc
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> acquire();
    // tx=1 发送; tx=0 接收（阻塞等待满或数据到来即完成）
    template <typename Alloc = co_wq::sys_taskalloc>
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc> transfer(uint8_t* data, size_t len, int tx);

    struct AcquireAwaiter {
        UsbCDCManager& self;
        cdc_usr*       tmp { nullptr };
        bool           result { false };
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };
        explicit AcquireAwaiter(UsbCDCManager& m) : self(m) { }
        bool await_ready()
        {
            if (self.handle_) {
                result = true;
                return true;
            }
            tmp = get_cdc_init();
            if (!tmp) {
                result = false;
                return true;
            }
            inner = new (inner_storage) SemAwaiter(cdc_usr_sem(tmp));
            if (inner->await_ready()) {
                self.handle_ = tmp;
                result       = true;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        bool await_resume()
        {
            self.handle_ = tmp;
            result       = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await() { return AcquireAwaiter(*this); }
    static_assert(std::is_trivially_destructible_v<SemAwaiter>, "SemReqAwaiter must remain trivially destructible");

    struct TransferAwaiter {
        UsbCDCManager& self;
        uint8_t*       data;
        size_t         len;
        int            tx;
        int            result_len { 0 };
        struct cdc_usr_node_cb : cdc_usr_node {
            explicit cdc_usr_node_cb(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1) { }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };
        cdc_usr_node_cb node;
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };
        TransferAwaiter(UsbCDCManager& m, uint8_t* d, size_t l, int t)
            : self(m), data(d), len(l), tx(t), node(cdc_usr_wq(m.handle_))
        {
        }
        bool await_ready()
        {
            if (!self.handle_) {
                result_len = 0;
                return true;
            }
            node.data    = data;
            node.dat_max = len;
            node.dat_cur = 0;
            node.func    = [](co_wq::worknode* pws) {
                auto* psess = static_cast<cdc_usr_node_cb*>(pws);
                psess->cpl_inotify.release();
            };
            uint32_t lk = lock_acquire();
            cdc_usr_async_setup_nolock(tx ? cdc_usr_list_tx(self.handle_) : cdc_usr_list_rx(self.handle_),
                                       &node,
                                       cdc_usr_ready(self.handle_),
                                       cdc_usr_dev(self.handle_),
                                       tx);
            lock_release(lk);
            inner = new (inner_storage) SemAwaiter(node.cpl_inotify);
            if (inner->await_ready()) {
                result_len = node.dat_cur;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = node.dat_cur;
            return result_len;
        }
    };
    TransferAwaiter transfer_await(uint8_t* data, size_t len, int tx) { return TransferAwaiter(*this, data, len, tx); }

private:
    cdc_usr* handle_;
};

// ============== 模板实现 ==============

// 初始化 USB CDC 设备（只初始化一次）
template <typename Alloc> co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>, Alloc> UsbCDCManager::acquire()
{
    co_return co_await acquire_await();
}

// USB CDC 传输/接收
template <typename Alloc>
co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>, Alloc>
UsbCDCManager::transfer(uint8_t* data, size_t len, int tx)
{
    co_return co_await transfer_await(data, len, tx);
}
