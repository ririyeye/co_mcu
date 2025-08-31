extern "C" {
#include "gd32e11x_misc.h"
}
#include "co_spi.hpp"
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

task_stat usr_sta;

struct usr_taskalloc {

public:
    void* alloc(std::size_t size)
    {
        usr_sta.malloc_cnt++;
        return malloc(size);
    }
    void dealloc(void* ptr, std::size_t sz) noexcept
    {
        (void)sz;
        usr_sta.free_cnt++;
        free(ptr);
    }
};

co_wq::Task<void, co_wq::Work_Promise<cortex_lock, void>, usr_taskalloc> u_send(UartManager& uart, void* buff, int len)
{
    co_await uart.uart_transfer(reinterpret_cast<uint8_t*>(buff), len, 1);

    co_await co_wq::DelayAwaiter(get_sys_timer(), 1000);

    co_return;
}

co_wq::Task<void, co_wq::Work_Promise<cortex_lock, void>, usr_taskalloc> test_task()
{
    auto uart = UartManager(0);
    bool succ = co_await uart.init();

    if (!succ) {
        co_return;
    }

    auto u1 = u_send(uart, (void*)"test", 4);
    auto u2 = u_send(uart, (void*)"pppp", 4);

    co_await when_all(u1, u2);

    co_return;
}

co_wq::Task<void, co_wq::Work_Promise<cortex_lock, void>> usb_task()
{
    auto cdc  = UsbCDCManager();
    bool succ = co_await cdc.init();

    if (!succ) {
        co_return;
    }

    while (1) {
        const char hello[] = "hello world\r\n";
        co_await cdc.transfer(reinterpret_cast<uint8_t*>(const_cast<char*>(hello)), sizeof(hello), 1);
        co_await co_wq::DelayAwaiter(get_sys_timer(), 1000);
    }

    co_return;
}

co_wq::Task<void, co_wq::Work_Promise<cortex_lock, void>> usb_recv_block_task(UsbCDCManager& cdc, char* pdata, int len)
{
    while (1) {
        co_await cdc.transfer(reinterpret_cast<uint8_t*>(pdata), len, 0);
    }

    co_return;
}

#define BLK_CNT 8
char usb_buff[BLK_CNT][64];

co_wq::Task<void, co_wq::Work_Promise<cortex_lock, void>> usb_recv_task()
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
        co_await co_wq::DelayAwaiter(get_sys_timer(), 1000);
    }

    co_return;
}

co_wq::Task<void, co_wq::Work_Promise<cortex_lock, void>> spi_task()
{
    auto spi  = SpiManager();
    bool succ = co_await spi.init(spi_is_master_not_slave | spi_is_8bit_not_16bit | spi_is_msb_not_lsb | spi_cp_mode_1);

    if (!succ) {
        co_return;
    }

    while (1) {
        char testpp[] = "123456";
        co_await spi.transfer((uint8_t*)testpp, nullptr, 6, 0);

        co_await co_wq::DelayAwaiter(get_sys_timer(), 1000);
    }

    co_return;
}

void usr_init(void)
{
    auto t = test_task();
    post_to(t, get_sys_workqueue());

    auto usb = usb_recv_task();
    post_to(usb, get_sys_workqueue());

    auto spi = spi_task();
    post_to(spi, get_sys_workqueue());
}

void usr_loop(void)
{
    while (true) {
        int sta = get_sys_workqueue().work_once();
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
