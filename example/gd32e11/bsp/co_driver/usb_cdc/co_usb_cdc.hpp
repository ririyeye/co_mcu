#include "worker.hpp"
#include <cstdint>

struct cdc_usr;
struct UsbCDCManager {
public:
    UsbCDCManager() : handle_(nullptr) { }
    ~UsbCDCManager();
    co_mcu::Task<bool, co_mcu::Work_Promise<bool>> init();
    co_mcu::Task<int, co_mcu::Work_Promise<int>>   transfer(uint8_t* data, size_t len, int tx);

private:
    cdc_usr* handle_;
};
