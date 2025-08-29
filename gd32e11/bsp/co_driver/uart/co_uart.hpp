#include "syswork.hpp"
#include "worker.hpp"
#include <cstdint>

struct uart_handle;
struct UartManager {
public:
    UartManager(int uart_num) : uart_num_(uart_num), handle_(nullptr) { }
    ~UartManager();
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>> init();
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>>   uart_transfer(uint8_t* data, size_t len, int tx);

private:
    int          uart_num_;
    uart_handle* handle_;
};
