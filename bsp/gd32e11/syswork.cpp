#include "syswork.hpp"
#include "workqueue.hpp"
#include <atomic>

using namespace co_wq;

struct executor_wq : workqueue<cortex_lock> {
    std::atomic_int trig_cnt = 0;

    explicit executor_wq()
    {
        INIT_LIST_HEAD(&ws_head);
        trig = [](struct workqueue* wq) {
            executor_wq* ewq = static_cast<executor_wq*>(wq);
            ewq->trig_cnt++;
        };
    }
};

executor_wq                    executor;
Timer_check_queue<cortex_lock> timer(executor);

workqueue<cortex_lock>& get_sys_workqueue(void)
{
    return executor;
}

Timer_check_queue<cortex_lock>& get_sys_timer(void)
{
    return timer;
}
