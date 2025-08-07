#include "systick.h"
#include "syswork.hpp"
#include "timer.hpp"

void usr_tick()
{
    get_sys_timer().tick_update();
}

void usr_init(void) { }

void usr_loop(void)
{
    while (true) {
        int sta = workqueue_once(&get_sys_workqueue());
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
