#pragma once
#include "co_base/task.hpp"
#include "lock.h"
#include "worker.hpp"
#include "workqueue.h"
#include <chrono>

namespace co_mcu {

struct sem_req : worknode {
    sem_req() { INIT_LIST_HEAD(&ws_node); }

    ~sem_req()
    {
        uint32_t lk = lock_acquire();
        list_del(&ws_node);
        lock_release(lk);
    }

    enum req_sta {
        REQ_OK = 0,
        REQ_FAIL,
        REQ_TIME_OUT,
    } req_sta;
};

struct semaphore : worknode {
private:
    workqueue& _executor;
    int        _cur_val;
    int        _max_val;
    list_head  acquire_list;

    void sem_chk_cb()
    {
        if (list_empty(&acquire_list)) {
            return; // nothing to do
        }

        worknode* pos;
        worknode* n;
        int       trig_flg = 0;

        uint32_t lk = lock_acquire();
        list_for_each_entry_safe (pos, n, &acquire_list, ws_node, worknode) {
            sem_req* req = static_cast<sem_req*>(pos);

            if (_cur_val > 0) {
                _cur_val--;
                list_del(&req->ws_node);
                workqueue_add_new_nolock(&_executor, req);
                req->req_sta = sem_req::REQ_OK;
                trig_flg     = 1;
            }
        }
        lock_release(lk);

        if (trig_flg) {
            workqueue_trig_once(&_executor);
        }
    }

public:
    explicit semaphore(workqueue& executor, int init_val, int max_val)
        : _executor(executor), _cur_val(init_val), _max_val(max_val)
    {
        INIT_LIST_HEAD(&ws_node);
        INIT_LIST_HEAD(&acquire_list);
        func = [](struct worknode* work) {
            semaphore* tcq = static_cast<semaphore*>(work);
            tcq->sem_chk_cb();
        };
    }

    void acquire(sem_req& sem_req)
    {
        uint32_t lk = lock_acquire();
        if (_cur_val > 0) {
            _cur_val--;
            workqueue_post(&_executor, &sem_req);
        } else {
            list_add_tail(&sem_req.ws_node, &acquire_list);
        }
        lock_release(lk);
    }

    void release()
    {
        int      post_flg = 0;
        uint32_t lk       = lock_acquire();
        if (_cur_val < _max_val) {
            _cur_val++;
            if (!list_empty(&acquire_list)) {
                post_flg = 1;
                sem_chk_cb(); // wake up one waiting request
            }
        }
        lock_release(lk);

        if (post_flg) {
            workqueue_post(&_executor, this);
        }
    }
};

template <class T>
struct SemReqAwaiter : sem_req {

    explicit SemReqAwaiter(semaphore& sem) : _semaphore(sem) { }

    std::coroutine_handle<> mCoroutine;
    semaphore&              _semaphore;

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<work_Promise<T>> coroutine) noexcept
    {
        mCoroutine = coroutine;
        INIT_LIST_HEAD(&ws_node);
        func = [](struct worknode* pws) {
            SemReqAwaiter* self = static_cast<SemReqAwaiter*>(pws);

            if (self->mCoroutine) {
                self->mCoroutine.resume();
            }
        };

        _semaphore.acquire(*this);
    };
    void await_resume() const noexcept { }
};

inline Task<void, work_Promise<void>> wait_sem(semaphore& sem)
{
    co_return co_await SemReqAwaiter<void>(sem);
}

}
