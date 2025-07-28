#include "co_base/task.hpp"
#include "usrlist.h"
#pragma once
namespace co_mcu {

struct back_Promise_base {
    LIST_HEAD(node);
};

void co_work_trig();
void co_work_trig(struct list_head& pnew);
struct back_Promise : Promise<void>, back_Promise_base {

    auto get_return_object() { return std::coroutine_handle<back_Promise>::from_promise(*this); }

    ~back_Promise() { list_del(&node); }

    void return_void() { co_work_trig(node); }

    auto yield_value(int)
    {
        co_work_trig(node);
        return std::suspend_always();
    }

    back_Promise& operator=(back_Promise&&) = delete;
};

list_head& co_g_work_head();

template <class T> void co_add_to_backwork(Task<T, back_Promise>& tk)
{
    auto& promise = tk.mCoroutine.promise();
    tk.detach(); // 放弃所有权，防止局部变量销毁时析构Task
    co_work_trig(promise.node);
}

}