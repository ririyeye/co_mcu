#include "syswork.hpp"
#include "workqueue.hpp"
#include <atomic>
struct executor_wq : co_wq::workqueue<cortex_lock> {
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

executor_wq       executor;
co_wq::Timer_check_queue<cortex_lock> timer(executor);

co_wq::workqueue<cortex_lock>& get_sys_workqueue(void)
{
    return executor;
}

co_wq::Timer_check_queue<cortex_lock>& get_sys_timer(void)
{
    return timer;
}
