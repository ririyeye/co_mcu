#pragma once
#include "co_base/task.hpp"
#include "lock.h"
#include "worker.hpp"
#include "workqueue.h"
#include <chrono>
#include <stdatomic.h>

namespace co_mcu {

struct notify_req_base{
    list_head notify_node;
};

struct notify_req : worknode, notify_req_base { };

struct inotify : worknode {
private:
    workqueue& _executor;
    list_head  acquire_list;
    atomic_int mNotify_one = 0;
    atomic_int mNotify_all = 0;

    // -1表示不限
    int inotify_num(int req_cnt)
    {
        notify_req_base* pos;
        notify_req_base* n;
        int         ret    = 0;
        uint32_t    lk     = lock_acquire();
        int         curcnt = req_cnt;
        list_for_each_entry_safe (pos, n, &acquire_list, notify_node, notify_req_base) {
            if (req_cnt > 0) {
                if (curcnt <= 0) {
                    break; // reach the limit
                }
                curcnt--;
            }

            notify_req* req = static_cast<notify_req*>(pos);
            workqueue_add_new_nolock(&_executor, req);
            ret++;
        }
        lock_release(lk);

        return ret;
    }

    void inotify_cb()
    {
        if (list_empty(&acquire_list)) {
            return; // nothing to do
        }

        int cnt = 0;
        if (mNotify_all > 0) {

            cnt = inotify_num(-1);

            mNotify_all = 0;
            mNotify_one = 0;
        } else {
            cnt         = inotify_num(mNotify_one);
            mNotify_one = 0;
        }

        if (cnt > 0) {
            workqueue_trig_once(&_executor);
        }
    }

public:
    explicit inotify(workqueue& executor) : _executor(executor)
    {
        INIT_LIST_HEAD(&ws_node);
        INIT_LIST_HEAD(&acquire_list);
        func = [](struct worknode* work) {
            inotify* tcq = static_cast<inotify*>(work);
            tcq->inotify_cb();
        };
    }

    void notify_all()
    {
        mNotify_all++;
        workqueue_post(&_executor, this);
    }

    void notify_one()
    {
        mNotify_one++;
        workqueue_post(&_executor, this);
    }

    void regist(notify_req& req)
    {
        uint32_t lk = lock_acquire();
        list_add_tail(&req.notify_node, &acquire_list);
        lock_release(lk);
    }

    void unregist(notify_req& req)
    {
        uint32_t lk = lock_acquire();
        list_del(&req.notify_node);
        lock_release(lk);
    }
};
struct NotifyReqAwaiter : notify_req {

    explicit NotifyReqAwaiter(inotify& inotify) : mInotify(inotify) { }

    ~NotifyReqAwaiter() { mInotify.unregist(*this); }

    std::coroutine_handle<> mCoroutine;
    inotify&                mInotify;

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<work_Promise<void>> coroutine) noexcept
    {
        mCoroutine = coroutine;
        INIT_LIST_HEAD(&ws_node);
        func = [](struct worknode* pws) {
            NotifyReqAwaiter* self = static_cast<NotifyReqAwaiter*>(pws);

            if (self->mCoroutine) {
                self->mCoroutine.resume();
            }
        };

        mInotify.regist(*this);
    };
    void await_resume() const noexcept { }
};

inline Task<void, work_Promise<void>> wait_inotify(inotify& sem)
{
    co_return co_await NotifyReqAwaiter(sem);
}
}
