#include "syswork.hpp"
#include "worker.hpp"
#include <cstdint>

typedef enum {
    spi_is_master_not_slave = 1 << 0,
    spi_is_8bit_not_16bit   = 1 << 1,
    spi_is_msb_not_lsb      = 1 << 2,

    spi_cpol = 1 << 3,
    spi_cpha = 1 << 4,

    spi_cp_mode_0 = 0,
    spi_cp_mode_1 = (0 | spi_cpha),
    spi_cp_mode_2 = (spi_cpol | 0),
    spi_cp_mode_3 = (spi_cpol | spi_cpha),

    spi_high_speed = 1 << 5,
} spi_mode_bit;

typedef enum {
    spi_cs_not_set_at_end  = 1 << 0,
    spi_cs_not_set_at_half = 1 << 1,
    spi_only_together      = 1 << 2,
    spi_short_cb           = 1 << 3,
    spi_fast_cb            = 1 << 4,
} spi_ctrl_bit;

struct spi_handle;
struct SpiManager {
public:
    SpiManager() : handle_(nullptr) { }
    ~SpiManager();
    co_wq::Task<bool, co_wq::Work_Promise<cortex_lock, bool>> init(uint32_t mode);
    co_wq::Task<int, co_wq::Work_Promise<cortex_lock, int>>
    transfer(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit);

private:
    spi_handle* handle_;
};
