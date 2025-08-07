#include "semaphore.hpp"
#include "syswork.hpp"

struct uart_handle;

struct uart_handle* uart_handle_get(int num);
co_mcu::semaphore&  uart_handle_get(struct uart_handle* puart);

co_mcu::Task<uart_handle*, co_mcu::work_Promise<uart_handle*>> wait_uart_hd(int num);

co_mcu::Task<int, co_mcu::work_Promise<int>>
uart_transfer(uart_handle* phd, const uint8_t* data, size_t len, int txflg);
