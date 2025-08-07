#include "co_base/task.hpp"
#include "workqueue.h"
#include <atomic>

#pragma once
namespace co_mcu {

struct work_Promise;

struct work_Promise : Promise<void>, worknode {

    auto get_return_object() { return std::coroutine_handle<work_Promise>::from_promise(*this); }

    virtual ~work_Promise() = default;

    struct workqueue* _excutor = nullptr;

    void post(workqueue* wq)
    {
        _excutor = wq;
        func = wk_cb;
        if (wq) {
            workqueue_post(wq, this);
        }
    }

    static void wk_cb(struct worknode* work)
    {
        work_Promise* promise = static_cast<work_Promise*>(work);
        auto          coro    = std::coroutine_handle<work_Promise>::from_promise(*promise);

        int doneflg = 0;

        if (!coro.done()) {
            coro.resume();
            if (coro.done()) {
                doneflg = 1;
            }
        } else {
            doneflg = 1;
        }

        if (doneflg) {
            // 协程已完成 销毁
            coro.destroy();
        }
    }

    void return_void()
    {
        func = wk_cb;
        workqueue_post(_excutor, this);
    }

    auto yield_value(int)
    {
        func = wk_cb;
        workqueue_post(_excutor, this);
        return std::suspend_always();
    }

    work_Promise& operator=(work_Promise&&) = delete;
};

static inline void post_to(Task<void, work_Promise>& tk, workqueue& executor)
{
    auto& promise = tk.mCoroutine.promise();
    INIT_LIST_HEAD(&promise.ws_node);
    tk.detach(); // 放弃所有权，防止局部变量销毁时析构Task
    promise.post(&executor);
}

}
