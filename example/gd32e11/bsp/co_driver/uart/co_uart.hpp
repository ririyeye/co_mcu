#include "worker.hpp"
#include <cstdint>

struct uart_handle;
struct UartManager {
public:
    UartManager(int uart_num) : uart_num_(uart_num), handle_(nullptr) { }
    ~UartManager();
    co_mcu::Task<bool, co_mcu::Work_Promise<bool>> init();
    co_mcu::Task<int, co_mcu::Work_Promise<int>>   uart_transfer(uint8_t* data, size_t len, int tx);

private:
    int          uart_num_;
    uart_handle* handle_;
};
