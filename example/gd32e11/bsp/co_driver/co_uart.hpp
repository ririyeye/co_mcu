#include "semaphore.hpp"
#include "syswork.hpp"
struct uart_session : worknode {
    uint8_t* buff;
    uint32_t len;
    uint32_t cur_len; // 当前已传输长度

    struct uart_handle* puart_handle;
};

struct uart_handle;

struct uart_handle* uart_handle_get(int num);
co_mcu::semaphore&  uart_handle_get(struct uart_handle* puart);

static inline co_mcu::Task<uart_handle*, co_mcu::work_Promise<uart_handle*>> wait_uart_hd(int num)
{
    struct uart_handle* puart = uart_handle_get(num);
    if (!puart) {
        co_return nullptr; // 如果没有获取到句柄，直接返回空指针
    }

    co_await co_mcu::SemReqAwaiter<uart_handle*>(uart_handle_get(puart));
    co_return puart;
}
