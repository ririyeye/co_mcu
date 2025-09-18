#pragma once
#include "co_adc_internal.hpp"
#include <cstdint>

using SemAwaiterAdc = co_wq::SemReqAwaiter<cortex_lock>;

// 协程版 ADC 管理器：支持编号构造，acquire 获取句柄，sample_once 异步单次采样
struct AdcManager {
public:
    explicit AdcManager(int num = 0) : adc_num_(num), handle_(nullptr) { }
    ~AdcManager();
    void release();

    // 获取底层句柄（只读）
    adc_handle* raw_handle() const { return handle_; }

    struct AcquireAwaiter {
        AdcManager& self;
        adc_handle* tmp { nullptr };
        bool        result { false };
        alignas(SemAwaiterAdc) unsigned char inner_storage[sizeof(SemAwaiterAdc)];
        SemAwaiterAdc* inner { nullptr };
        explicit AcquireAwaiter(AdcManager& m) : self(m) { }
        bool await_ready()
        {
            if (self.handle_) {
                result = true;
                return true;
            }
            tmp = adc_handle_get_init(self.adc_num_);
            if (!tmp) {
                result = false;
                return true;
            }
            inner = new (inner_storage) SemAwaiterAdc(adc_handle_sem(tmp));
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
    static_assert(std::is_trivially_destructible_v<SemAwaiterAdc>, "SemReqAwaiter must remain trivially destructible");

    // 单次采样 Awaiter：传入通道与采样时间，返回采样值（int；负数为错误）
    struct SampleAwaiter {
        AdcManager& self;
        uint8_t     chan_idx;
        uint32_t    sample_time; // GD32 ADC_SAMPLETIME_*
        uint16_t    result_val { 0 };
        int         result_code { 0 };

        struct co_adc_session : adc_session {
            explicit co_adc_session(co_wq::workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1)
            {
                INIT_LIST_HEAD(&ws_node);
            }
            co_wq::Semaphore<cortex_lock> cpl_inotify;
        };

        alignas(co_adc_session) unsigned char node_storage[sizeof(co_adc_session)];
        alignas(SemAwaiterAdc) unsigned char inner_storage[sizeof(SemAwaiterAdc)];
        SemAwaiterAdc* inner { nullptr };

        SampleAwaiter(AdcManager& m, uint8_t ch, uint32_t st) : self(m), chan_idx(ch), sample_time(st) { }

        bool await_ready()
        {
            if (!self.handle_) {
                result_code = -1;
                return true;
            }
            auto* node = reinterpret_cast<co_adc_session*>(node_storage);
            new (node) co_adc_session(adc_handle_wq(self.handle_));
            node->chan_idx    = chan_idx;
            node->sample_time = sample_time;
            node->func        = [](co_wq::worknode* p) {
                auto* s = static_cast<co_adc_session*>(p);
                s->cpl_inotify.release();
            };
            adc_enqueue_session(*self.handle_, *node);
            inner = new (inner_storage) SemAwaiterAdc(node->cpl_inotify);
            if (inner->await_ready()) {
                result_val  = node->result;
                result_code = static_cast<int>(result_val);
                return true;
            }
            return false;
        }
        void await_suspend(std::coroutine_handle<> h) { inner->await_suspend(h); }
        int  await_resume()
        {
            auto* node  = reinterpret_cast<co_adc_session*>(node_storage);
            result_val  = node->result;
            result_code = static_cast<int>(result_val);
            return result_code;
        }
    };

    SampleAwaiter sample_once_await(uint8_t chan_idx, uint32_t sample_time)
    {
        return SampleAwaiter(*this, chan_idx, sample_time);
    }

private:
    int         adc_num_ { 0 };
    adc_handle* handle_;
};
