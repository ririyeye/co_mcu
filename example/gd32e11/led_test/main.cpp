#include "co_uart.hpp"
#include "systick.h"
#include "syswork.hpp"
#include "timer.hpp"
void usr_tick()
{
    get_sys_timer().tick_update();
}

co_mcu::Task<void, co_mcu::work_Promise<void>> test_task()
{
    auto hd = co_await wait_uart_hd(0);

    if(!hd) {
        co_return;
    }

    while (1) {
        const char hello[] = "hello world\r\n";
        co_await uart_transfer(hd, (const uint8_t*)hello, sizeof(hello), 1);
        co_await co_mcu::DelayAwaiter(get_sys_timer(), 1000);
    }
    

    co_return;
}

void usr_init(void)
{
    auto t = test_task();
    post_to(t, get_sys_workqueue());
}

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
