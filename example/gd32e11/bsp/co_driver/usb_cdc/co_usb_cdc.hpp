#include "syswork.hpp"

struct cdc_usr;
struct UsbCDCManager {
public:
    UsbCDCManager(int uart_num) : uart_num_(uart_num), handle_(nullptr) { }
    ~UsbCDCManager();
    co_mcu::Task<bool, co_mcu::Work_Promise<bool>> init();
    co_mcu::Task<int, co_mcu::Work_Promise<int>>   transfer(uint8_t* data, size_t len, int tx);

private:
    int      uart_num_;
    cdc_usr* handle_;
};
