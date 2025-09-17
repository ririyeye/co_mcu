#pragma once
#include "co_usb_cdc_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

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
            if (!self.handle_ && tmp) {
                self.handle_ = tmp;
            }
            result = (self.handle_ != nullptr);
            return result;
        }
    };
    AcquireAwaiter acquire_await() { return AcquireAwaiter(*this); }
    static_assert(std::is_trivially_destructible_v<SemAwaiter>, "SemReqAwaiter must remain trivially destructible");

    // 统一模板收发 awaiter：N 段，N=1 等价单次
    template <int N = 2> struct TransferAwaiter {
        UsbCDCManager& self;
        // 内部持有数组拷贝，避免包装函数中局部数组悬垂
        uint8_t* datas[N];
        size_t   lens[N];
        int      txs[N];
        int      result_len { 0 };
        struct cdc_usr_node_cb : cdc_usr_node {
            explicit cdc_usr_node_cb(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1) { }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };
        alignas(cdc_usr_node_cb) unsigned char nodes_storage[sizeof(cdc_usr_node_cb) * N];
        alignas(SemAwaiter) unsigned char inner_storage[sizeof(SemAwaiter)];
        SemAwaiter* inner { nullptr };
        TransferAwaiter(UsbCDCManager& m, uint8_t* (&d)[N], size_t (&l)[N], int (&t)[N]) : self(m)
        {
            for (int i = 0; i < N; ++i) {
                datas[i] = d[i];
                lens[i]  = l[i];
                txs[i]   = t[i];
                new (&nodes_ptr()[i]) cdc_usr_node_cb(cdc_usr_wq(self.handle_));
            }
        }
        bool await_ready()
        {
            if (!self.handle_) {
                result_len = 0;
                return true;
            }
            for (int i = 0; i < N; ++i) {
                auto& node   = nodes_ptr()[i];
                node.data    = datas[i];
                node.dat_max = lens[i];
                node.dat_cur = 0;
                node.func    = nullptr;
            }
            // 最后一段回调唤醒
            nodes_ptr()[N - 1].func = [](co_wq::worknode* pws) {
                auto* psess = static_cast<cdc_usr_node_cb*>(pws);
                psess->cpl_inotify.release();
            };
            // 依次提交
            uint32_t lk = lock_acquire();
            for (int i = 0; i < N; ++i) {
                cdc_usr_async_setup_nolock(txs[i] ? cdc_usr_list_tx(self.handle_) : cdc_usr_list_rx(self.handle_),
                                           &nodes_ptr()[i],
                                           cdc_usr_ready(self.handle_),
                                           cdc_usr_dev(self.handle_),
                                           txs[i]);
            }
            lock_release(lk);
            inner = new (inner_storage) SemAwaiter(nodes_ptr()[N - 1].cpl_inotify);
            if (inner->await_ready()) {
                result_len = nodes_ptr()[N - 1].dat_cur;
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            result_len = nodes_ptr()[N - 1].dat_cur;
            return result_len;
        }

    private:
        inline cdc_usr_node_cb* nodes_ptr() { return reinterpret_cast<cdc_usr_node_cb*>(nodes_storage); }
    };

    // 单段便捷接口
    TransferAwaiter<1> transfer_await(uint8_t* data, size_t len, int tx)
    {
        uint8_t* datas[1] = { data };
        size_t   lens[1]  = { len };
        int      txs[1]   = { tx };
        return TransferAwaiter<1>(*this, datas, lens, txs);
    }

    // 多段接口
    template <int N = 2> TransferAwaiter<N> transfer_multi_await(uint8_t* (&datas)[N], size_t (&lens)[N], int (&txs)[N])
    {
        return TransferAwaiter<N>(*this, datas, lens, txs);
    }

private:
    int      cdc_num_ { 0 };
    cdc_usr* handle_;
};
