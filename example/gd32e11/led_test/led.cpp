extern "C" {
#include "gd32e11x_misc.h"
}
#include "co_uart.hpp"
#include "co_usb_cdc.hpp"
#include "systick.h"
#include "syswork.hpp"
#include "timer.hpp"
#include "when_all.hpp"

void usr_tick()
{
    get_sys_timer().tick_update();
}

co_mcu::Task<void, co_mcu::Work_Promise<void>> u_send(UartManager& uart, char* buff, int len)
{
    co_await uart.uart_transfer(reinterpret_cast<uint8_t*>(buff), len, 1);

    co_await co_mcu::DelayAwaiter(get_sys_timer(), 1000);

    co_return;
}

co_mcu::Task<void, co_mcu::Work_Promise<void>> test_task()
{
    auto uart = UartManager(0);
    bool succ = co_await uart.init();

    if (!succ) {
        co_return;
    }

    auto u1 = u_send(uart, "test", 4);
    auto u2 = u_send(uart, "pppp", 4);

    co_await when_all(u1, u2);

    co_return;
}

co_mcu::Task<void, co_mcu::Work_Promise<void>> usb_task()
{
    auto cdc  = UsbCDCManager();
    bool succ = co_await cdc.init();

    if (!succ) {
        co_return;
    }

    while (1) {
        const char hello[] = "hello world\r\n";
        co_await cdc.transfer(reinterpret_cast<uint8_t*>(const_cast<char*>(hello)), sizeof(hello), 1);
        co_await co_mcu::DelayAwaiter(get_sys_timer(), 1000);
    }

    co_return;
}

co_mcu::Task<void, co_mcu::Work_Promise<void>> usb_recv_block_task(UsbCDCManager& cdc, char* pdata, int len)
{
    while (1) {
        co_await cdc.transfer(reinterpret_cast<uint8_t*>(pdata), len, 0);
    }

    co_return;
}

#define BLK_CNT 8
char usb_buff[BLK_CNT][64];

co_mcu::Task<void, co_mcu::Work_Promise<void>> usb_recv_task()
{
    auto cdc  = UsbCDCManager();
    bool succ = co_await cdc.init();

    if (!succ) {
        co_return;
    }

    for (int i = 0; i < BLK_CNT; i++) {
        auto u1 = usb_recv_block_task(cdc, usb_buff[i], sizeof(usb_buff[i]));
        post_to(u1, get_sys_workqueue());
    }

    while (1) {
        co_await co_mcu::DelayAwaiter(get_sys_timer(), 1000);
    }

    co_return;
}

void usr_init(void)
{
    auto t = test_task();
    post_to(t, get_sys_workqueue());

    auto usb = usb_recv_task();
    post_to(usb, get_sys_workqueue());
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
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    systick_config();
    usr_init();
    while (1) {
        usr_loop();
    }
    return 0;
}
