#include "syswork.hpp"
#include <atomic>
struct executor_wq : workqueue {
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

executor_wq               executor;
co_mcu::Timer_check_queue timer(executor);

struct workqueue& get_sys_workqueue(void)
{
    return executor;
}

co_mcu::Timer_check_queue& get_sys_timer(void)
{
    return timer;
}
