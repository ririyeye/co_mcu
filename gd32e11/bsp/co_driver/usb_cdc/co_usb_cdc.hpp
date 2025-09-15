#pragma once
#include "co_usb_cdc_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"

using SemAwaiter = co_wq::SemReqAwaiter<cortex_lock>;

struct cdc_usr;
// USB CDC 协程管理器：支持编号构造（当前仅实现 0）。
//  - UsbCDCManager(num) 传入编号；num!=0 时 acquire() 将返回 false。
//  - acquire() 初始化底层 USB CDC 设备（一次）。
//  - transfer() 异步发送/接收，内部使用队列 + 中断回调唤醒。
struct UsbCDCManager {
public:
    explicit UsbCDCManager(int num = 0) : cdc_num_(num), handle_(nullptr) { }
    ~UsbCDCManager();
    void release();
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
            tmp = get_cdc_init_by_num(self.cdc_num_);
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
    int      cdc_num_ { 0 };
    cdc_usr* handle_;
};
