
#include "co_base/task.hpp"
#include "sys_workqueue.h"
#include "workqueue.h"
#include <coroutine>
#include "task.h"

#pragma once

namespace co_mcu {

struct DelayAwaiter {
    work_struct_delayed     delay_work;
    uint32_t                wait_ms;
    std::coroutine_handle<> mCoroutine;

    DelayAwaiter(uint32_t ms)
    {
        INIT_WORK(&delay_work.ws, timer_callback);
        wait_ms = ms;
    }

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<Promise<void>> coroutine) noexcept
    {
        mCoroutine = coroutine;
        sys_queue_work_delayed(&delay_work, wait_ms);
    }

    void await_resume() const noexcept { }

    static void timer_callback(struct work_struct* pws)
    {
        auto* delay = container_of(pws, work_struct_delayed, ws);
        auto* self  = container_of(delay, DelayAwaiter, delay_work);
        if (self->mCoroutine) {
            self->mCoroutine.resume();
        }
    }
};

inline Task<void, Promise<void>> delay_ms(uint32_t ms)
{
    co_return co_await DelayAwaiter(ms);
}

}
