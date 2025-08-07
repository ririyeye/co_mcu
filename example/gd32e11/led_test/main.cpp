#include "systick.h"
#include "timer.hpp"

struct executor_wq : workqueue {
    std::atomic_int trig_flg = 0;

    explicit executor_wq()
    {
        INIT_LIST_HEAD(&ws_head);
        trig = [](struct workqueue* wq) {
            executor_wq* ewq = static_cast<executor_wq*>(wq);
            ewq->trig_flg    = 1;
        };
    }
};

executor_wq               executor;
co_mcu::timer_check_queue timer(executor);

void usr_tick()
{
    timer.tick_update();
}

void usr_init(void) { }

void usr_loop(void)
{
    while (true) {
        int sta = workqueue_once(&executor);
        if (!sta) {
            return;
        }
    }
}

extern "C" int main()
{
    systick_config();
    usr_init();
    while (1) {
        usr_loop();
    }
    return 0;
}
