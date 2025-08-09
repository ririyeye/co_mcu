#include "co_base/task.hpp"
#include "workqueue.h"

#pragma once
namespace co_mcu {

template <class T = void> struct Base_Work_Promise : Promise<T>, worknode {
    virtual ~Base_Work_Promise() = default;

    struct workqueue* _excutor = nullptr;

    void post(workqueue* wq)
    {
        _excutor = wq;
        func     = wk_cb;
        if (wq) {
            workqueue_post(wq, this);
        }
    }

    void post(void) { post(this->_excutor); }

    static void wk_cb(struct worknode* work)
    {
        auto* promise = static_cast<Base_Work_Promise*>(work);
        auto  coro    = std::coroutine_handle<Base_Work_Promise>::from_promise(*promise);

        if (!coro.done()) {
            coro.resume();
        }

        if (coro.done()) {
            coro.destroy();
        }
    }

    Base_Work_Promise& operator=(Base_Work_Promise&&) = delete;
};

template <class T = void> struct Work_Promise : Base_Work_Promise<T> {
    auto get_return_object() { return std::coroutine_handle<Work_Promise>::from_promise(*this); }

    void return_value(T&& ret)
    {
        Base_Work_Promise<T>::post();
        Promise<T>::return_value(std::move(ret));
    }

    void return_value(T const& ret)
    {
        Base_Work_Promise<T>::post();
        Promise<T>::return_value(ret);
    }
};

template <> struct Work_Promise<void> : Base_Work_Promise<void> {
    auto get_return_object() { return std::coroutine_handle<Work_Promise>::from_promise(*this); }

    void return_void() noexcept { Base_Work_Promise<>::post(); }
};

template <class T> static inline void post_to(Task<T, Work_Promise<T>>& tk, workqueue& executor)
{
    auto& promise = tk.mCoroutine.promise();
    INIT_LIST_HEAD(&promise.ws_node);
    tk.detach(); // 放弃所有权，防止局部变量销毁时析构Task
    promise.post(&executor);
}
}
