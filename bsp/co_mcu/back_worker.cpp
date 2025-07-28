
#include "back_worker.hpp"
#include "portmacro.h"
#include "usrlist.h"
#include <atomic>
#include <stddef.h>
namespace co_mcu {

struct co_work {
    std::atomic_bool   set_flg = false;

    int wk;
    LIST_HEAD(co_work_head);

    explicit co_work(void) {

    };
    static void co_work_cb(struct work_struct* work)
    {
        co_work* pnode = container_of(work, co_work, wk);
        pnode->set_flg = 0;

        while (1) {
            int cnt = pnode->co_work_once(pnode->co_work_head);
            if (cnt == 0) {
                break;
            }
        }
    }

    void trig_once()
    {
        bool unset = false;
        if (!atomic_compare_exchange_strong(&set_flg, &unset, true)) {
            return;
        }

    }

    int co_work_once(list_head& head)
    {
        int                cnt  = 0;
        back_Promise_base* base = list_first_entry_or_null(&head, back_Promise_base, node);
        if (base) {
            list_del(&base->node);
            cnt++;
            back_Promise* promise = static_cast<back_Promise*>(base);
            auto          coro    = std::coroutine_handle<back_Promise>::from_promise(*promise);

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
                // 协程已完成，从链表移除并销毁
                coro.destroy();
            }
        }
        return cnt;
    }
};

co_work _co_work;

void co_work_trig()
{
    _co_work.trig_once();
}

void co_work_trig(struct list_head& pnew)
{
    vPortEnterCritical();
    list_del(&pnew);
    list_add_tail(&pnew, &_co_work.co_work_head);
    vPortExitCritical();
    co_work_trig();
}

}
