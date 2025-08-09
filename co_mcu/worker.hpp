#include "co_base/task.hpp"
#include "workqueue.h"
#include <atomic>

#pragma once
namespace co_mcu {

template <class T = void> struct Work_Promise : Promise<T>, worknode {

    auto get_return_object() { return std::coroutine_handle<Work_Promise>::from_promise(*this); }

    virtual ~Work_Promise() = default;

    struct workqueue* _excutor = nullptr;

    void post(workqueue* wq)
    {
        _excutor = wq;
        func     = wk_cb;
        if (wq) {
            workqueue_post(wq, this);
        }
    }

    static void wk_cb(struct worknode* work)
    {
        Work_Promise* promise = static_cast<Work_Promise*>(work);
        auto          coro    = std::coroutine_handle<Work_Promise>::from_promise(*promise);

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

    void return_value(T&& ret)
    {
        func = wk_cb;
        workqueue_post(_excutor, this);
        Promise<T>::return_value(std::move(ret));
    }

    void return_value(T const& ret)
    {
        func = wk_cb;
        workqueue_post(_excutor, this);
        Promise<T>::return_value(std::move(ret));
    }

    auto yield_value(int)
    {
        func = wk_cb;
        workqueue_post(_excutor, this);
        return std::suspend_always();
    }

    Work_Promise& operator=(Work_Promise&&) = delete;
};

template <> struct Work_Promise<void> : Promise<void>, worknode {
    auto get_return_object() { return std::coroutine_handle<Work_Promise>::from_promise(*this); }

    virtual ~Work_Promise() = default;

    struct workqueue* _excutor = nullptr;

    void post(workqueue* wq)
    {
        _excutor = wq;
        func     = wk_cb;
        if (wq) {
            workqueue_post(wq, this);
        }
    }

    static void wk_cb(struct worknode* work)
    {
        Work_Promise* promise = static_cast<Work_Promise*>(work);
        auto          coro    = std::coroutine_handle<Work_Promise>::from_promise(*promise);

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

    void return_void() noexcept
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

    Work_Promise& operator=(Work_Promise&&) = delete;
};

template <class T> static inline void post_to(Task<T, Work_Promise<T>>& tk, workqueue& executor)
{
    auto& promise = tk.mCoroutine.promise();
    INIT_LIST_HEAD(&promise.ws_node);
    tk.detach(); // 放弃所有权，防止局部变量销毁时析构Task
    promise.post(&executor);
}
}
