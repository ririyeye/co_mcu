#include "worker.hpp"
#include <cstdint>
#include "syswork.hpp"

struct cdc_usr;
struct UsbCDCManager {
public:
    UsbCDCManager() : handle_(nullptr) { }
    ~UsbCDCManager();
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>> init();
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>>  transfer(uint8_t* data, size_t len, int tx);

private:
    cdc_usr* handle_;
};
